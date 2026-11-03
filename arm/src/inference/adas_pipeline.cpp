#include "inference/adas_pipeline.h"
#include "core/road_geometry.h"
#include "core/tracked_overlays.h"
#include "inference/neural_models.h"
#include "inference/neural_schedule.h"
#include "core/view_tracker.h"
#include <QCoreApplication>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <opencv2/imgproc.hpp>

namespace adas {
namespace {
bool valid_frame(const FramePacket &frame) {
  return frame && frame.width > 0 && frame.height > 0 && frame.stride >= frame.width * 2 &&
    static_cast<size_t>(frame.stride) * (frame.height - 1) + static_cast<size_t>(frame.width) * 2 <= frame.pixels->bytes.size();
}
struct PendingFrame {
  std::mutex mutex;
  std::condition_variable changed;
  FramePacket pending;
  std::atomic<uint64_t> dropped{0};
  bool stopping = false, has_submitted = false;
  uint64_t consumed_generation = 0, consumed_sequence = 0;
  uint64_t generation = 0, sequence = 0;
  bool accept(FramePacket packet) { // Caller holds mutex.
    if (stopping || !valid_frame(packet)) return false;
    if (has_submitted && (packet.stamp.generation < generation ||
        (packet.stamp.generation == generation && packet.stamp.sequence <= sequence))) return false;
    has_submitted = true; generation = packet.stamp.generation; sequence = packet.stamp.sequence;
    if (pending && (pending.stamp.generation != consumed_generation || pending.stamp.sequence > consumed_sequence)) ++dropped;
    pending = std::move(packet); changed.notify_one(); return true;
  }
};
} // namespace

struct DetectionWorker::State : PendingFrame {
  QString model_root;
  DetectionMode mode=DetectionMode::Adas;
  GroundCalibration calibration;
  std::array<Roi,3> light_rois{};
  TurnSignal intent=TurnSignal::None;
  uint64_t config_revision=0;
};
DetectionWorker::DetectionWorker(QObject *parent):QThread(parent),state_(new State){
  qRegisterMetaType<VisionFrameResult>("VisionFrameResult");
}
DetectionWorker::~DetectionWorker(){stop();}
void DetectionWorker::set_model_root(QString root){
  std::lock_guard<std::mutex> lock(state_->mutex);
  if(state_->model_root==root)return;
  state_->model_root=std::move(root);++state_->config_revision;
  state_->changed.notify_all();
}
void DetectionWorker::set_mode(DetectionMode mode){
  std::lock_guard<std::mutex> lock(state_->mutex);
  if(state_->mode==mode)return;
  state_->mode=mode;++state_->config_revision;state_->pending={};state_->changed.notify_all();
}
void DetectionWorker::configure(GroundCalibration calibration,std::array<Roi,3> light_rois){
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->calibration=std::move(calibration);state_->light_rois=light_rois;
  ++state_->config_revision;state_->pending={};state_->changed.notify_all();
}
void DetectionWorker::set_intent(TurnSignal signal){
  std::lock_guard<std::mutex> lock(state_->mutex);state_->intent=signal;state_->changed.notify_all();
}
void DetectionWorker::submit(FramePacket packet){
  std::lock_guard<std::mutex> lock(state_->mutex);
  if(state_->mode==DetectionMode::Adas)state_->accept(std::move(packet));
}
uint64_t DetectionWorker::dropped_frames()const{return state_->dropped.load();}
void DetectionWorker::stop(){
  {std::lock_guard<std::mutex> lock(state_->mutex);state_->stopping=true;state_->pending={};}
  state_->changed.notify_all();if(QThread::currentThread()!=this)wait();
}
void DetectionWorker::run(){
  NeuralSchedule schedule;ViewTracker tracker;std::unique_ptr<NeuralModels> models;
  uint64_t generation=0,revision=0;bool have_generation=false;QString loaded_root;
  for(;;){
    FramePacket packet;QString root;GroundCalibration calibration;
    std::array<Roi,3> rois;TurnSignal intent;uint64_t config_revision;
    {
      std::unique_lock<std::mutex> lock(state_->mutex);
      state_->changed.wait(lock,[&]{return state_->stopping||bool(state_->pending);});
      if(state_->stopping)return;
      packet=state_->pending;root=state_->model_root;calibration=state_->calibration;
      rois=state_->light_rois;intent=state_->intent;config_revision=state_->config_revision;
    }
    if(!have_generation||generation!=packet.stamp.generation||revision!=config_revision){
      schedule.reset();tracker.reset();generation=packet.stamp.generation;
      revision=config_revision;have_generation=true;
    }
    const auto current=[&]{
      return !state_->stopping&&state_->mode==DetectionMode::Adas&&
        state_->generation==packet.stamp.generation&&state_->config_revision==config_revision;
    };
    if(!packet.stamp.static_image&&monotonic_seconds()-packet.stamp.receive_s>.5){
      std::unique_lock<std::mutex> lock(state_->mutex);
      if(!state_->stopping)state_->changed.wait_for(lock,std::chrono::milliseconds(10));
      continue;
    }
    if(root.isEmpty())root=QCoreApplication::applicationDirPath();
    if(!models||loaded_root!=root){
      models=std::make_unique<NeuralModels>();models->initialize(root);loaded_root=root;
      schedule.reset();continue; // Model loading may have outlived the selected frame.
    }
    const auto task=schedule.next(packet.layout,intent,monotonic_seconds(),packet.stamp.sequence);
    if(!task){
      std::unique_lock<std::mutex> lock(state_->mutex);
      if(!state_->stopping)state_->changed.wait_for(lock,std::chrono::milliseconds(10));
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      if(!current())continue;
      state_->consumed_generation=packet.stamp.generation;state_->consumed_sequence=packet.stamp.sequence;
    }
    const auto role=task.kind==NeuralTask::Vehicle?SceneModelRole::Vehicle:SceneModelRole::Traffic;
    auto result=models->scene(packet,task.view,rois,{0,0,1,1},role);
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      if(!current())continue;
    }
    if(role==SceneModelRole::Vehicle){
      auto &objects=result.vision.observation;
      tracker.update(objects);
      for(auto &object:objects.obstacles){
        if(object.predicted)object.distance_m=-1;
        else if(task.view==ViewId::Front&&calibration.is_compatible(objects.width,objects.height))
          object.distance_m=float(calibration.distance(object.box));
      }
      sync_tracking_overlays(result.vision,monotonic_seconds());
    }
    // Traffic observations keep their own components and never enter the
    // primary person/vehicle tracker. Publish under the transition lock.
    std::lock_guard<std::mutex> lock(state_->mutex);
    if(current())emit result_ready(std::move(result.vision));
  }
}
struct GeometryWorker::State : PendingFrame {
  bool enabled=false;
  uint64_t config_revision=0,published_generation=0,published_sequence=0;
  bool has_published=false,has_consumed=false;
};
GeometryWorker::GeometryWorker(QObject*parent):QThread(parent),state_(new State){qRegisterMetaType<GeometryObservation>("adas::GeometryObservation");}
GeometryWorker::~GeometryWorker(){stop();}
void GeometryWorker::set_enabled(bool enabled){
  std::lock_guard<std::mutex> lock(state_->mutex);
  if(state_->enabled==enabled)return;
  state_->enabled=enabled;++state_->config_revision;state_->pending={};state_->changed.notify_all();
}
void GeometryWorker::configure(GroundCalibration){
  // A configuration transition cancels in-flight lane work.
  std::lock_guard<std::mutex> lock(state_->mutex);
  ++state_->config_revision;state_->pending={};state_->changed.notify_all();
}
void GeometryWorker::submit(FramePacket packet){
  std::lock_guard<std::mutex> lock(state_->mutex);
  if(state_->enabled)state_->accept(std::move(packet));
}
uint64_t GeometryWorker::dropped_frames()const{return state_->dropped.load();}
void GeometryWorker::stop(){
  {std::lock_guard<std::mutex> lock(state_->mutex);state_->stopping=true;state_->pending={};++state_->config_revision;}
  state_->changed.notify_all();if(QThread::currentThread()!=this)wait();
}
void GeometryWorker::run(){
  RoadGeometry geometry;
  using Clock=std::chrono::steady_clock;
  const auto period=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1./15));
  Clock::time_point next_due{};
  const auto fresh=[](const Stamp &stamp){
    const double age=monotonic_seconds()-stamp.receive_s;
    return stamp.static_image||(std::isfinite(age)&&age>=0&&age<=.5);
  };
  for(;;){
    FramePacket packet;uint64_t revision;
    {
      std::unique_lock<std::mutex> lock(state_->mutex);
      state_->changed.wait(lock,[&]{return state_->stopping||(state_->enabled&&state_->pending&&
        (!state_->has_consumed||state_->pending.stamp.generation!=state_->consumed_generation||state_->pending.stamp.sequence>state_->consumed_sequence));});
      if(state_->stopping)return;
      if(Clock::now()<next_due){state_->changed.wait_until(lock,next_due);continue;}
      packet=state_->pending;revision=state_->config_revision;
      state_->consumed_generation=packet.stamp.generation;state_->consumed_sequence=packet.stamp.sequence;
      state_->has_consumed=true;
      next_due=Clock::now()+period;
    }
    GeometryObservation result;
    result.components=GeometryObservation::Lane;result.stamp=packet.stamp;
    const auto rect=view_rect(packet.layout,ViewId::Front,packet.width,packet.height);
    result.width=rect.w;result.height=rect.h;result.status=Status::Failed;
    const bool was_fresh=fresh(packet.stamp);
    const auto started=Clock::now();
    if(was_fresh){
      try {
        cv::Mat packed(rect.h,rect.w,CV_8UC2,
          const_cast<unsigned char*>(packet.pixels->bytes.data())+size_t(rect.y)*packet.stride+rect.x*2,
          size_t(packet.stride));
        cv::Mat bgr;cv::cvtColor(packed,bgr,cv::COLOR_BGR5652BGR);
        result=geometry.analyze_lane(bgr,packet.stamp);
      } catch(const std::exception &) {
        result.status=Status::Failed;
      }
      result.process_ms=float(std::chrono::duration<double,std::milli>(Clock::now()-started).count());
    }
    {
      // Publishing shares the transition lock so a finished validity check
      // cannot race disable/configure. Slots must only enqueue this value.
      std::lock_guard<std::mutex> lock(state_->mutex);
      if(state_->pending&&state_->pending.stamp.generation==packet.stamp.generation&&
          state_->pending.stamp.sequence==packet.stamp.sequence)state_->pending={};
      if(state_->stopping)return; // The local packet remains owned until here.
      const bool newer=!state_->has_published||packet.stamp.generation>state_->published_generation||
        (packet.stamp.generation==state_->published_generation&&packet.stamp.sequence>state_->published_sequence);
      if(state_->enabled&&revision==state_->config_revision&&packet.stamp.generation==state_->generation&&
          packet.stamp.sequence==state_->consumed_sequence&&newer&&was_fresh&&fresh(packet.stamp)){
        state_->published_generation=packet.stamp.generation;state_->published_sequence=packet.stamp.sequence;state_->has_published=true;
        emit result_ready(std::move(result));
      }
    }
  }
}
}
