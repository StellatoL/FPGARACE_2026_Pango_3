#include "inference/adas_pipeline.h"
#include "inference/neural_models.h"
#include <QCoreApplication>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <set>
#include <thread>

namespace {
std::mutex mutex;
std::condition_variable changed;
bool blocked=false;
uint64_t blocked_generation=0,entered_generation=0;
int initializations=0;
struct Call { adas::Stamp stamp; adas::ViewId view; adas::SceneModelRole role; std::thread::id thread; };
std::vector<Call> calls;
std::vector<VisionFrameResult> observations;

adas::FramePacket frame(uint64_t generation,uint64_t sequence,
                        adas::Layout layout=adas::Layout::Mosaic) {
  constexpr int width=128,height=96,stride=272;
  auto memory=std::make_shared<adas::FrameStorage>(stride*height);
  const double now=adas::monotonic_seconds();
  return {memory,{generation,sequence,now,now,false},width,height,stride,layout};
}
bool wait_for(const std::function<bool()> &condition) {
  std::unique_lock<std::mutex> lock(mutex);
  return changed.wait_for(lock,std::chrono::seconds(3),condition);
}
void connect_results(adas::DetectionWorker &worker) {
  QObject::connect(&worker,&adas::DetectionWorker::result_ready,&worker,[](VisionFrameResult result) {
    std::lock_guard<std::mutex> lock(mutex);
    observations.push_back(std::move(result));changed.notify_all();
  },Qt::DirectConnection);
}
void reset_observations() {
  std::lock_guard<std::mutex> lock(mutex);observations.clear();calls.clear();entered_generation=0;
}
bool has_pair(uint64_t generation,uint64_t sequence) {
  int count=0;
  for(const auto &result:observations)
    if(result.stamp.generation==generation&&result.stamp.sequence==sequence)++count;
  return count>=2;
}
void block_generation(uint64_t generation) {
  std::lock_guard<std::mutex> lock(mutex);blocked=true;blocked_generation=generation;entered_generation=0;
}
void unblock() {
  std::lock_guard<std::mutex> lock(mutex);blocked=false;changed.notify_all();
}
}

// Substitute only the SDK-facing model calls. Scheduling, threads, mailbox,
// generations, tracking and distance calibration use production code.
namespace adas {
struct NeuralModels::State {};
NeuralModels::NeuralModels():state_(new State){}
NeuralModels::~NeuralModels()=default;
bool NeuralModels::initialize(const QString&) {
  std::lock_guard<std::mutex> lock(mutex);++initializations;return true;
}
QString NeuralModels::description()const{return "test substitute only";}
NeuralSceneResult NeuralModels::scene(const FramePacket &packet,ViewId view,
    const std::array<Roi,3>&,const Roi &crop,SceneModelRole role) {
  assert(crop.x==0&&crop.y==0&&crop.w==1&&crop.h==1);
  {
    std::unique_lock<std::mutex> lock(mutex);
    calls.push_back({packet.stamp,view,role,std::this_thread::get_id()});
    entered_generation=packet.stamp.generation;changed.notify_all();
    changed.wait(lock,[&]{return !blocked||packet.stamp.generation!=blocked_generation;});
  }
  NeuralSceneResult out;out.role=role;
  auto &result=out.vision;result.model_ready=true;result.stamp=packet.stamp;result.view=view;
  auto &objects=result.observation;objects.stamp=packet.stamp;objects.view=view;
  const auto rect=view_rect(packet.layout,view,packet.width,packet.height);
  objects.width=rect.w;objects.height=rect.h;
  if(role==SceneModelRole::Vehicle) {
    objects.components=ObjectObservation::Combined;
    objects.traffic_status=objects.vehicle_status=Status::Valid;
    objects.obstacles={{1,-1,{20,10,30,35},.9f,-1},{10,-1,{40,10,50,35},.9f,-1}};
    if(packet.stamp.generation==6&&packet.stamp.sequence==4)objects.obstacles.clear();
  } else {
    assert(role==SceneModelRole::Traffic);
    objects.components=ObjectObservation::Crosswalk|(view==ViewId::Front?ObjectObservation::Signal:0);
    objects.crosswalk_status=Status::Valid;
    if(view==ViewId::Front)objects.signal_status=Status::Valid;
    OverlayBoxResult auxiliary;auxiliary.class_id=12;auxiliary.label="person";
    result.overlays.push_back(auxiliary);
  }
  return out;
}
}

namespace {
void test_four_views_and_roles() {
  reset_observations();adas::DetectionWorker worker;connect_results(worker);
  worker.start();worker.submit(frame(1,1));
  assert(wait_for([]{return observations.size()==8;}));worker.stop();
  std::lock_guard<std::mutex> lock(mutex);
  assert(calls.size()==8&&observations.size()==8);
  unsigned vehicle_mask=0,traffic_mask=0;std::set<int64_t> identities;
  const auto executor=calls.front().thread;
  assert(executor!=std::this_thread::get_id());
  for(size_t i=0;i<observations.size();++i) {
    const auto &result=observations[i];const auto &objects=result.observation;
    assert(result.mode==DetectionMode::Adas&&objects.width==64&&objects.height==48);
    assert(calls[i].thread==executor&&calls[i].view==result.view);
    assert(calls[i].role==(i%2?adas::SceneModelRole::Traffic:adas::SceneModelRole::Vehicle));
    assert(result.stamp.receive_s==calls[i].stamp.receive_s&&objects.stamp.source_s==calls[i].stamp.source_s);
    if(i%2==0) {
      assert(objects.components==adas::ObjectObservation::Combined&&objects.obstacles.size()==2);
      for(const auto &object:objects.obstacles)assert(object.track_id>=0&&identities.insert(object.track_id).second);
      vehicle_mask|=1u<<int(result.view);
    } else {
      assert(objects.components&adas::ObjectObservation::Crosswalk);
      assert(objects.obstacles.empty()&&result.overlays.size()==1);
      assert(result.overlays[0].class_id==12&&result.overlays[0].track_id<0);
      traffic_mask|=1u<<int(result.view);
    }
  }
  assert(vehicle_mask==15&&traffic_mask==15);
}
void test_generation_mailbox_and_ownership() {
  reset_observations();adas::DetectionWorker worker;connect_results(worker);
  block_generation(2);worker.start();
  auto pending=frame(2,1);std::weak_ptr<const adas::FrameStorage> ownership=pending.pixels;
  worker.submit(pending);pending={};
  assert(wait_for([]{return entered_generation==2;}));assert(!ownership.expired());
  worker.submit(frame(3,1,adas::Layout::Single));worker.submit(frame(3,2,adas::Layout::Single));
  worker.submit(frame(3,2));worker.submit(frame(3,1));worker.submit(frame(1,999));
  assert(worker.dropped_frames()>0);unblock();
  assert(wait_for([]{return has_pair(3,2);}));
  worker.stop();assert(!worker.isRunning()&&ownership.expired());
  std::lock_guard<std::mutex> lock(mutex);
  assert(observations.size()==2);
  for(const auto &result:observations) {
    assert(result.stamp.generation==3&&result.stamp.sequence==2&&result.view==adas::ViewId::Front);
    assert(result.observation.width==128&&result.observation.height==96);
  }
}
void test_tracking_and_calibration() {
  reset_observations();adas::DetectionWorker worker;connect_results(worker);
  adas::GroundCalibration calibration;
  assert(calibration.configure({{{0,0},{127,0},{127,95},{0,95}}},10,50,5,128,96));
  worker.configure(calibration,{});worker.start();
  for(uint64_t sequence=1;sequence<=4;++sequence) {
    worker.submit(frame(6,sequence,adas::Layout::Single));
    assert(wait_for([sequence]{return has_pair(6,sequence);}));
  }
  worker.stop();std::lock_guard<std::mutex> lock(mutex);
  std::array<int64_t,2> identities{{-1,-1}};bool measured=false,predicted=false;
  for(const auto &result:observations) {
    if(result.observation.components!=adas::ObjectObservation::Combined)continue;
    const auto &objects=result.observation.obstacles;assert(objects.size()==2);
    for(size_t i=0;i<objects.size();++i) {
      if(identities[i]<0)identities[i]=objects[i].track_id;
      assert(objects[i].track_id==identities[i]);
      if(result.stamp.sequence==3){assert(!objects[i].predicted&&objects[i].distance_m>0);measured=true;}
      if(result.stamp.sequence==4){assert(objects[i].predicted&&objects[i].distance_m==-1);predicted=true;}
    }
  }
  assert(measured&&predicted);
}
void test_raw_mode_transitions_and_stop() {
  reset_observations();adas::DetectionWorker worker;connect_results(worker);
  worker.set_mode(DetectionMode::RawVideo);worker.start();
  auto raw=frame(20,1);std::weak_ptr<const adas::FrameStorage> raw_owner=raw.pixels;
  worker.submit(raw);raw={};assert(raw_owner.expired());
  worker.set_mode(DetectionMode::Adas);block_generation(21);
  auto old=frame(21,1);std::weak_ptr<const adas::FrameStorage> old_owner=old.pixels;
  worker.submit(old);old={};assert(wait_for([]{return entered_generation==21;}));
  worker.set_mode(DetectionMode::RawVideo);worker.set_mode(DetectionMode::Adas);
  worker.submit(frame(21,2,adas::Layout::Single));unblock();
  assert(wait_for([]{return has_pair(21,2);}));assert(old_owner.expired());
  block_generation(22);auto stopping=frame(22,1);
  std::weak_ptr<const adas::FrameStorage> stopping_owner=stopping.pixels;
  worker.submit(stopping);stopping={};assert(wait_for([]{return entered_generation==22;}));
  assert(!stopping_owner.expired());
  std::atomic<bool> stop_entered{false},stopped{false};
  std::thread stopper([&]{stop_entered=true;worker.stop();stopped=true;});
  while(!stop_entered.load())std::this_thread::yield();
  // stop must wait for the SDK-facing call to return and relinquish its packet.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));assert(!stopped.load());
  unblock();stopper.join();assert(stopped.load()&&stopping_owner.expired()&&!worker.isRunning());
  std::lock_guard<std::mutex> lock(mutex);
  for(const auto &result:observations)assert(result.stamp.generation==21&&result.stamp.sequence==2);
}
}
int main(int argc,char **argv) {
  QCoreApplication app(argc,argv);
  static_assert(int(DetectionMode::RawVideo)==2&&int(DetectionMode::Adas)==3);
  test_four_views_and_roles();test_generation_mailbox_and_ownership();
  test_tracking_and_calibration();test_raw_mode_transitions_and_stop();
  assert(initializations==4);
  std::cout<<"single NPU worker dual-role/views/tracking/ownership/generation/raw/stop passed\n";
}
