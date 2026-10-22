#include "inference/adas_pipeline.h"
#include "common/vision_common.h"
#include "core/plate_recognizer.h"
#include "inference/neural_models.h"
#include "inference/neural_schedule.h"
#include "core/traffic_scene_detector.h"
#include "core/view_tracker.h"
#include <QCoreApplication>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <sstream>

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
VisionFrameResult plate_overlays(const PlateFrameResult &plates) {
  VisionFrameResult result;
  result.mode = DetectionMode::PlateRecognition; result.model_ready = plates.model_ready;
  result.error_message = plates.error_message;
  struct Style { const char *key; int r, g, b; };
  const Style types[] = {{u8"教练车",255,200,0},{u8"警车",255,255,255},
    {u8"军车",255,255,255},{u8"使馆车",60,60,60},{u8"港澳车",60,60,60},
    {u8"货车",255,200,0},{u8"新能源",0,200,80},{u8"标准车牌",0,100,255}};
  const Style colors[] = {{u8"蓝",0,100,255},{u8"绿",0,200,80},{u8"黄",255,200,0},
    {u8"白",220,220,220},{u8"黑",60,60,60}};
  for (const auto &plate : plates.plates) {
    OverlayBoxResult box;
    box.left = plate.left; box.top = plate.top; box.right = plate.right; box.bottom = plate.bottom;
    box.confidence = plate.confidence;
    const Style *chosen = nullptr;
    for (const auto &style : types) if (plate.plate_type.find(style.key) != std::string::npos) { chosen = &style; break; }
    if (!chosen) for (const auto &style : colors) if (plate.color == style.key) { chosen = &style; break; }
    if (chosen) { box.color_r = chosen->r; box.color_g = chosen->g; box.color_b = chosen->b; }
    std::ostringstream label;
    if (!plate.text.empty()) label << plate.text;
    if (!plate.plate_type.empty()) label << ' ' << plate.plate_type;
    if (label.tellp() == 0) label << "plate";
    label << ' ' << plate.confidence; box.label = label.str();
    result.overlays.push_back(std::move(box));
  }
  return result;
}
} // namespace

struct DetectionWorker::State : PendingFrame {
  QString model_root;DetectionMode mode=DetectionMode::Adas;GroundCalibration calibration;
  std::array<Roi,3> light_rois{};Roi navigation{0,0,.32,.30};TurnSignal intent=TurnSignal::None;
};
DetectionWorker::DetectionWorker(QObject *parent):QThread(parent),state_(new State){
  qRegisterMetaType<VisionFrameResult>("VisionFrameResult");qRegisterMetaType<GeometryObservation>("adas::GeometryObservation");
}
DetectionWorker::~DetectionWorker(){stop();}
void DetectionWorker::set_model_root(QString root){std::lock_guard<std::mutex> l(state_->mutex);state_->model_root=std::move(root);}
void DetectionWorker::set_mode(DetectionMode mode){std::lock_guard<std::mutex> l(state_->mutex);state_->mode=mode;state_->pending={};state_->changed.notify_all();}
void DetectionWorker::configure(GroundCalibration c,std::array<Roi,3> r,Roi n){std::lock_guard<std::mutex> l(state_->mutex);state_->calibration=std::move(c);state_->light_rois=r;state_->navigation=n;}
void DetectionWorker::set_intent(TurnSignal signal){std::lock_guard<std::mutex> l(state_->mutex);state_->intent=signal;state_->changed.notify_all();}
void DetectionWorker::submit(FramePacket packet){std::lock_guard<std::mutex> l(state_->mutex);if(state_->mode!=DetectionMode::RawVideo)state_->accept(std::move(packet));}
uint64_t DetectionWorker::dropped_frames()const{return state_->dropped.load();}
void DetectionWorker::stop(){
  {std::lock_guard<std::mutex> l(state_->mutex);state_->stopping=true;state_->pending={};}
  state_->changed.notify_all();if(QThread::currentThread()!=this)wait();
}
void DetectionWorker::run(){
  PlateRecognizer plates;TrafficSceneDetector legacy;bool plate_attempted=false,legacy_attempted=false;
  NeuralSchedule schedule;ViewTracker tracker;std::unique_ptr<NeuralModels> models;
  uint64_t generation=0;bool have_generation=false;QString loaded_root;
  DetectionMode previous=DetectionMode::RawVideo;
  for(;;){
    FramePacket packet;DetectionMode mode;QString root;GroundCalibration calibration;
    std::array<Roi,3> rois;Roi nav;TurnSignal intent;
    {
      std::unique_lock<std::mutex> lock(state_->mutex);
      state_->changed.wait(lock,[&]{return state_->stopping||bool(state_->pending);});
      if(state_->stopping)return;
      packet=state_->pending;mode=state_->mode;root=state_->model_root;calibration=state_->calibration;
      rois=state_->light_rois;nav=state_->navigation;intent=state_->intent;
      if(mode!=DetectionMode::Adas)state_->pending={};
    }
    if(mode==DetectionMode::RawVideo)continue;
    if(!have_generation||generation!=packet.stamp.generation||previous!=mode){
      schedule.reset();tracker.reset();legacy.reset_tracking();generation=packet.stamp.generation;have_generation=true;previous=mode;
    }
    const auto current=[&]{std::lock_guard<std::mutex> l(state_->mutex);return !state_->stopping&&state_->mode==mode&&state_->generation==packet.stamp.generation;};
    if(root.isEmpty())root=QCoreApplication::applicationDirPath();
    if(mode!=DetectionMode::Adas){
      VisionFrameResult result;
      try {
        if(mode==DetectionMode::PlateRecognition){if(!plate_attempted){plates.initialize(root.toStdString());plate_attempted=true;}result=plate_overlays(plates.infer_rgb565(packet.pixels->bytes.data(),packet.width,packet.height,packet.stride));}
        else {if(!legacy_attempted){legacy.initialize(root.toStdString());legacy_attempted=true;}result=legacy.infer_rgb565(packet.pixels->bytes.data(),packet.width,packet.height,packet.stride);}
      }catch(const std::exception&e){result.error_message=e.what();}
      result.mode=mode;result.stamp=packet.stamp;result.observation.stamp=packet.stamp;
      if(current())emit result_ready(std::move(result));
      continue;
    }
    if(!packet.stamp.static_image&&monotonic_seconds()-packet.stamp.receive_s>.5){
      std::unique_lock<std::mutex> l(state_->mutex);if(!state_->stopping)state_->changed.wait_for(l,std::chrono::milliseconds(10));continue;
    }
    auto task=schedule.next(packet.layout,intent,monotonic_seconds(),packet.stamp.sequence);
    if(!task){
      std::unique_lock<std::mutex> l(state_->mutex);if(!state_->stopping)state_->changed.wait_for(l,std::chrono::milliseconds(10));continue;
    }
    {std::lock_guard<std::mutex> l(state_->mutex);state_->consumed_generation=packet.stamp.generation;state_->consumed_sequence=packet.stamp.sequence;}
    if(!models||loaded_root!=root){models=std::make_unique<NeuralModels>();models->initialize(root);loaded_root=root;}
    if(task.kind==NeuralTask::Lane){auto result=models->lane(packet);if(current())emit geometry_ready(std::move(result));continue;}
    Roi crop{0,0,1,1};
    if(task.kind==NeuralTask::Navigation)crop=nav;
    if(task.kind==NeuralTask::Signal){
      double x=1,y=1,right=0,bottom=0;
      for(const auto&r:rois){x=std::min(x,r.x);y=std::min(y,r.y);right=std::max(right,r.x+r.w);bottom=std::max(bottom,r.y+r.h);}
      crop={x,y,right-x,bottom-y};
    }
    auto result=models->scene(packet,task.view,rois,nav,crop);
    if(!current())continue;
    if(task.kind==NeuralTask::Navigation){emit geometry_ready(std::move(result.navigation));continue;}
    auto &objects=result.vision.observation;
    if(task.kind==NeuralTask::Signal){
      objects.components=ObjectObservation::Signal;objects.obstacles.clear();result.vision.overlays.clear();
      emit result_ready(std::move(result.vision));continue;
    }
    tracker.update(objects);
    for(auto &o:objects.obstacles)if(task.view==ViewId::Front&&calibration.is_compatible(objects.width,objects.height))o.distance_m=float(calibration.distance(o.box));
    emit result_ready(std::move(result.vision));
    // Navigation and signal ROIs own their confirmation clocks. Full-frame
    // empty detections must not repeatedly cancel a positive small ROI target.
  }
}
// Retain source compatibility for legacy diagnostic callers. The production
// controller uses DetectionWorker::geometry_ready; this worker never reads pixels.
struct GeometryWorker::State : PendingFrame {bool enabled=false;};
GeometryWorker::GeometryWorker(QObject*parent):QThread(parent),state_(new State){qRegisterMetaType<GeometryObservation>("adas::GeometryObservation");}
GeometryWorker::~GeometryWorker(){stop();}
void GeometryWorker::set_enabled(bool enabled){std::lock_guard<std::mutex> l(state_->mutex);state_->enabled=enabled;}
void GeometryWorker::configure(GroundCalibration,Roi){}
void GeometryWorker::update_objects(ObjectObservation){}
void GeometryWorker::submit(FramePacket){ }
uint64_t GeometryWorker::dropped_frames()const{return 0;}
void GeometryWorker::stop(){{std::lock_guard<std::mutex> l(state_->mutex);state_->stopping=true;}state_->changed.notify_all();if(QThread::currentThread()!=this)wait();}
void GeometryWorker::run(){std::unique_lock<std::mutex> l(state_->mutex);state_->changed.wait(l,[&]{return state_->stopping;});}
}
