#include "inference/adas_pipeline.h"
#include "inference/neural_models.h"
#include "core/road_geometry.h"
#include <QCoreApplication>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

// These unused NPU adapters let this test link the real CPU worker without an
// RKNN runtime. The road geometry and RGB565 conversion are real implementations.
namespace adas {
struct NeuralModels::State {};
NeuralModels::NeuralModels():state_(new State){} NeuralModels::~NeuralModels()=default;
bool NeuralModels::initialize(const QString&){return false;}
QString NeuralModels::description()const{return {};}
NeuralSceneResult NeuralModels::scene(const FramePacket&,ViewId,const std::array<Roi,3>&,const Roi&,SceneModelRole){std::abort();}
}

namespace {
void check(bool condition,const char *message){
  if(!condition){std::cerr<<"FAIL: "<<message<<'\n';std::exit(1);}
}
cv::Mat lane_image(){
  cv::Mat image(480,640,CV_8UC3,cv::Scalar(45,45,45));
  auto lx=[](int y){return cvRound(285-(y-210)*.55);};
  auto rx=[](int y){return cvRound(355+(y-210)*.55);};
  cv::line(image,{lx(210),210},{lx(475),475},cv::Scalar(255,255,255),8);
  for(int y=210;y<470;y+=58)
    cv::line(image,{rx(y),y},{rx(std::min(y+30,475)),std::min(y+30,475)},cv::Scalar(0,230,230),8);
  return image;
}
adas::FramePacket frame(uint64_t generation,uint64_t sequence,adas::Layout layout=adas::Layout::Single,
                         bool large=false,bool static_image=true){
  cv::Mat image=lane_image();
  if(large)cv::resize(image,image,{6144,4608},0,0,cv::INTER_NEAREST);
  if(layout==adas::Layout::Mosaic){
    cv::Mat mosaic(image.rows*2,image.cols*2,CV_8UC3,cv::Scalar(255,255,255));
    image.copyTo(mosaic(cv::Rect(0,0,image.cols,image.rows)));image=mosaic;
  }
  cv::Mat packed;cv::cvtColor(image,packed,cv::COLOR_BGR2BGR565);
  // Include row padding, so front-view conversion must honor the packet stride.
  const int stride=packed.cols*2+32;
  auto memory=std::make_shared<adas::FrameStorage>(size_t(stride)*packed.rows);
  for(int y=0;y<packed.rows;++y)std::copy_n(packed.ptr<unsigned char>(y),packed.cols*2,memory->bytes.data()+size_t(y)*stride);
  const double now=adas::monotonic_seconds();
  return {memory,{generation,sequence,now,now,static_image},packed.cols,packed.rows,stride,layout};
}
class Results {
public:
  explicit Results(adas::GeometryWorker &worker){
    QObject::connect(&worker,&adas::GeometryWorker::result_ready,&worker,[this](adas::GeometryObservation result){
      std::lock_guard<std::mutex> lock(mutex);items.push_back(std::move(result));changed.notify_all();
    },Qt::DirectConnection);
  }
  bool wait(uint64_t generation,uint64_t sequence){
    std::unique_lock<std::mutex> lock(mutex);
    return changed.wait_for(lock,std::chrono::seconds(3),[&]{return std::any_of(items.begin(),items.end(),[&](const auto &r){return r.stamp.generation==generation&&r.stamp.sequence==sequence;});});
  }
  std::vector<adas::GeometryObservation> copy(){std::lock_guard<std::mutex> lock(mutex);return items;}
private:
  std::mutex mutex;std::condition_variable changed;std::vector<adas::GeometryObservation> items;
};
bool processing(const adas::FramePacket &packet){
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(std::chrono::steady_clock::now()<until){
    // The caller, latest mailbox, and current computation each retain a share.
    if(packet.pixels.use_count()>=3)return true;
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  return false;
}
void test_lane_only_api(){
  auto image=lane_image();
  std::vector<cv::Point> arrow={{320,45},{275,95},{300,95},{300,175},{340,175},{340,95},{365,95}};
  cv::fillPoly(image,std::vector<std::vector<cv::Point>>{arrow},cv::Scalar(255,90,0));
  const double now=adas::monotonic_seconds();adas::Stamp stamp{1,1,now,now,true};
  adas::RoadGeometry geometry;
  const auto result=geometry.analyze_lane(image,stamp);
  check(result.components==adas::GeometryObservation::Lane&&result.status==adas::Status::Valid,"lane API emits only valid lane evidence");
  check(!result.nav.detected&&!result.speed.valid&&result.speed.features==0,"lane API produces no navigation or optical flow");
  check(result.lane.left_type==adas::LineType::Solid&&result.lane.right_type==adas::LineType::Dashed&&
        result.lane.left_x<200&&result.lane.right_x>440,"lane API preserves solid and dashed lane fitting");
  for(const cv::Mat &invalid:{cv::Mat{},cv::Mat(16,16,CV_8UC3),cv::Mat(480,640,CV_8UC1)}){
    const auto failed=geometry.analyze_lane(invalid,stamp);
    check(failed.status==adas::Status::Failed&&failed.components==adas::GeometryObservation::Lane,"invalid lane-only input fails without other components");
  }
}
void test_lane_and_crop(){
  adas::GeometryWorker worker;Results results(worker);worker.set_enabled(true);worker.start();
  worker.submit(frame(0,0));check(results.wait(0,0),"enabled CPU worker accepts initial zero generation and sequence");
  worker.submit(frame(1,2,adas::Layout::Mosaic));check(results.wait(1,2),"mosaic front crop emits lane result");
  worker.stop();auto all=results.copy();check(all.size()==2,"one result per accepted frame");
  for(const auto &result:all){
    check(result.components==adas::GeometryObservation::Lane,"only lane component is produced");
    check(result.status==adas::Status::Valid&&result.width==640&&result.height==480,"local front coordinates retain native resolution");
    check(result.lane.left_type==adas::LineType::Solid&&result.lane.right_type==adas::LineType::Dashed,"real solid and dashed paint survives RGB565");
    check(result.lane.left_x<200&&result.lane.right_x>440&&result.lane.bottom_y>470,"lane coordinates are local native pixels");
    check(!result.nav.detected&&!result.speed.valid&&result.speed.features==0,"lane worker does not infer navigation or optical flow");
    check(result.process_ms>0,"lane result includes CPU processing duration");
  }
  check(std::abs(all[0].lane.right_x-all[1].lane.right_x)<1,"other mosaic views do not affect front lane evidence");
}
void test_latest_duplicates_and_disabled(){
  adas::GeometryWorker worker;Results results(worker);worker.start();
  auto disabled=frame(1,1);std::weak_ptr<const adas::FrameStorage> disabled_owner=disabled.pixels;
  worker.submit(disabled);disabled={};check(disabled_owner.expired(),"disabled worker retains no submitted pixels");
  worker.set_enabled(true);
  // Before starting computation, rapid submissions replace one mailbox entry.
  for(uint64_t sequence=1;sequence<=25;++sequence)worker.submit(frame(2,sequence));
  check(results.wait(2,25),"overload eventually processes the newest frame");
  const auto size=results.copy().size();worker.submit(frame(2,25));worker.submit(frame(2,24));worker.submit(frame(1,999));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  check(results.copy().size()==size,"same sequence, reversed sequence and old generation never re-run");
  check(worker.dropped_frames()>0,"latest mailbox counts overwritten unprocessed frames");
  worker.set_enabled(false);auto ignored=frame(3,1);std::weak_ptr<const adas::FrameStorage> ignored_owner=ignored.pixels;
  worker.submit(ignored);ignored={};check(ignored_owner.expired(),"disable immediately stops accepting work");worker.stop();
}
void test_inflight_generation_config_enable_and_ownership(){
  adas::GeometryWorker worker;Results results(worker);worker.set_enabled(true);worker.start();
  auto old=frame(10,1,adas::Layout::Single,true);std::weak_ptr<const adas::FrameStorage> ownership=old.pixels;
  worker.submit(old);check(processing(old),"real CPU computation retains its current frame");
  old={};worker.submit(frame(11,1));check(results.wait(11,1),"new generation runs after old in-flight work");
  check(ownership.expired(),"completed old work releases its pixels");
  for(const auto&r:results.copy())check(r.stamp.generation!=10,"old in-flight generation cannot publish");

  auto configured=frame(12,1,adas::Layout::Single,true);worker.submit(configured);
  check(processing(configured),"config test reaches in-flight CPU processing");worker.configure({});
  worker.submit(frame(12,2));check(results.wait(12,2),"new configuration accepts the next frame");
  for(const auto&r:results.copy())check(r.stamp.generation!=12||r.stamp.sequence!=1,"configure invalidates prior in-flight revision");

  auto toggled=frame(13,1,adas::Layout::Single,true);worker.submit(toggled);
  check(processing(toggled),"enable test reaches in-flight CPU processing");worker.set_enabled(false);worker.set_enabled(true);
  worker.submit(frame(13,2));check(results.wait(13,2),"reenable accepts fresh work");
  for(const auto&r:results.copy())check(r.stamp.generation!=13||r.stamp.sequence!=1,"disable and reenable cannot revive an older revision");
  worker.stop();
}
void test_freshness_invalid_and_stop(){
  adas::GeometryWorker worker;Results results(worker);worker.set_enabled(true);worker.start();
  auto stale=frame(20,1,adas::Layout::Single,false,false);stale.stamp.receive_s-=1;worker.submit(stale);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  check(results.copy().empty(),"stale realtime frame is rejected even without a replacement");
  auto invalid=frame(20,2);invalid.stride=1;worker.submit(invalid);
  worker.submit(frame(20,3));check(results.wait(20,3),"valid static frame is accepted after stale and invalid frames");
  auto before=results.copy();check(before.size()==1,"stale realtime and malformed pixels never produce fresh geometry");
  auto pending=frame(21,1,adas::Layout::Single,true);std::weak_ptr<const adas::FrameStorage> ownership=pending.pixels;
  worker.submit(pending);check(processing(pending),"stop test reaches in-flight computation");pending={};
  check(!ownership.expired(),"current CPU frame remains owned after producer release");
  worker.stop();check(!worker.isRunning()&&ownership.expired(),"stop waits until CPU computation releases its pixels");
  check(results.copy().size()==before.size(),"stopping invalidates in-flight publication");
}
}
int main(int argc,char **argv){
  QCoreApplication app(argc,argv);cv::setNumThreads(1);
  test_lane_only_api();test_lane_and_crop();test_latest_duplicates_and_disabled();
  test_inflight_generation_config_enable_and_ownership();test_freshness_invalid_and_stop();
  std::cout<<"CPU lane-only worker crop/latest/ownership/revisions/freshness/stop passed\n";
}
