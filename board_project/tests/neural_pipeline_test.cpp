#include "inference/adas_pipeline.h"
#include "inference/neural_models.h"
#include "core/traffic_scene_detector.h"
#include "core/plate_recognizer.h"
#include <QCoreApplication>
#include <cassert>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <thread>
#include <iostream>
namespace {
std::mutex mutex;std::condition_variable changed;bool block=false,entered=false;
uint64_t entered_generation=0;int legacy_calls=0,initializations=0;std::vector<VisionFrameResult> observations;
std::vector<adas::GeometryObservation> geometry;
adas::FramePacket frame(uint64_t gen,uint64_t seq,adas::Layout layout=adas::Layout::Mosaic){
  auto memory=std::make_shared<adas::FrameStorage>(128*96*2);
  return {memory,{gen,seq,adas::monotonic_seconds(),adas::monotonic_seconds(),false},128,96,256,layout};
}
bool wait_for(const std::function<bool()> &condition){
  std::unique_lock<std::mutex> l(mutex);return changed.wait_for(l,std::chrono::seconds(3),condition);
}
}
namespace adas {
struct NeuralModels::State {bool lane_enabled=true;};
NeuralModels::NeuralModels():state_(new State){}NeuralModels::~NeuralModels()=default;
bool NeuralModels::initialize(const QString&root){++initializations;state_->lane_enabled=root!="scene-only";return true;}
bool NeuralModels::lane_configured()const{return state_->lane_enabled;}
QString NeuralModels::description()const{return "test substitute only";}
NeuralSceneResult NeuralModels::scene(const FramePacket&p,ViewId v,const std::array<Roi,3>&,const Roi&,const Roi&){
  {std::unique_lock<std::mutex> l(mutex);entered=true;entered_generation=p.stamp.generation;changed.notify_all();changed.wait(l,[&]{return !block||p.stamp.generation!=2;});}
  NeuralSceneResult out;auto &r=out.vision;r.mode=DetectionMode::Adas;r.model_ready=true;r.stamp=p.stamp;r.view=v;
  auto &o=r.observation;o.components=ObjectObservation::All;o.stamp=p.stamp;o.view=v;
  auto rect=view_rect(p.layout,v,p.width,p.height);o.width=rect.w;o.height=rect.h;
  o.traffic_status=o.vehicle_status=o.roadblock_status=o.signal_status=Status::Valid;
  o.obstacles={{1,-1,{20,10,30,35},.9f,-1},{10,-1,{40,10,50,35},.9f,-1},{11,-1,{2,2,8,8},.9f,-1}};
  out.navigation.stamp=p.stamp;out.navigation.components=GeometryObservation::Navigation;out.navigation.status=Status::Valid;
  return out;
}
GeometryObservation NeuralModels::lane(const FramePacket&p){GeometryObservation o;o.stamp=p.stamp;o.components=GeometryObservation::Lane;o.status=lane_configured()?Status::Valid:Status::Missing;return o;}
}
TrafficSceneDetector::TrafficSceneDetector()=default;TrafficSceneDetector::~TrafficSceneDetector()=default;
bool TrafficSceneDetector::initialize(const std::string&){return true;}
void TrafficSceneDetector::reset_tracking(){}
VisionFrameResult TrafficSceneDetector::infer_rgb565(const unsigned char*,int,int,int,bool,bool){++legacy_calls;return {};}
VisionFrameResult TrafficSceneDetector::infer_vehicle_rgb565(const unsigned char*,int,int,int){++legacy_calls;return {};}
PlateRecognizer::PlateRecognizer()=default;PlateRecognizer::~PlateRecognizer()=default;
bool PlateRecognizer::initialize(const std::string&){return true;}
PlateFrameResult PlateRecognizer::infer_rgb565(const unsigned char*,int,int,int){++legacy_calls;return {};}
int main(int argc,char**argv){
 QCoreApplication app(argc,argv);
 adas::DetectionWorker worker;
 QObject::connect(&worker,&adas::DetectionWorker::result_ready,&worker,[](VisionFrameResult r){std::lock_guard<std::mutex> l(mutex);observations.push_back(std::move(r));changed.notify_all();},Qt::DirectConnection);
 QObject::connect(&worker,&adas::DetectionWorker::geometry_ready,&worker,[](adas::GeometryObservation r){std::lock_guard<std::mutex> l(mutex);geometry.push_back(std::move(r));changed.notify_all();},Qt::DirectConnection);
 worker.start();worker.submit(frame(1,1));
 assert(wait_for([]{unsigned mask=0;for(const auto&r:observations)if(r.observation.components==adas::ObjectObservation::All)mask|=1u<<int(r.view);return mask==15&&!geometry.empty();}));
 {std::lock_guard<std::mutex> l(mutex);for(const auto&r:observations)if(r.observation.components==adas::ObjectObservation::All){assert(r.observation.width==64&&r.observation.height==48);assert(r.observation.obstacles.size()==3);for(const auto&o:r.observation.obstacles)assert(o.track_id>=0);}observations.clear();geometry.clear();block=true;entered=false;}
 auto pending=frame(2,1);std::weak_ptr<const adas::FrameStorage> ownership=pending.pixels;
 worker.submit(pending);pending={};assert(wait_for([]{return entered_generation==2;}));assert(!ownership.expired());
 {std::lock_guard<std::mutex> l(mutex);observations.clear();geometry.clear();}
 worker.submit(frame(3,1,adas::Layout::Single));worker.submit(frame(3,2,adas::Layout::Single));
 {std::lock_guard<std::mutex> l(mutex);block=false;changed.notify_all();}
 assert(wait_for([]{return !observations.empty();}));
 {std::lock_guard<std::mutex> l(mutex);for(const auto&r:observations){assert(r.stamp.generation==3);assert(r.observation.width==128&&r.observation.height==96);assert(r.view==adas::ViewId::Front);}}
 worker.submit(frame(1,999));
 worker.set_mode(DetectionMode::RawVideo);worker.submit(frame(4,1));
 worker.stop();assert(!worker.isRunning()&&ownership.expired());assert(legacy_calls==0&&initializations==1);
 {std::lock_guard<std::mutex> l(mutex);observations.clear();geometry.clear();}
 adas::DetectionWorker scene_only;scene_only.set_model_root("scene-only");
 QObject::connect(&scene_only,&adas::DetectionWorker::result_ready,&scene_only,[](VisionFrameResult r){std::lock_guard<std::mutex> l(mutex);observations.push_back(std::move(r));changed.notify_all();},Qt::DirectConnection);
 QObject::connect(&scene_only,&adas::DetectionWorker::geometry_ready,&scene_only,[](adas::GeometryObservation r){std::lock_guard<std::mutex> l(mutex);geometry.push_back(std::move(r));changed.notify_all();},Qt::DirectConnection);
 scene_only.start();scene_only.submit(frame(5,1));
 assert(wait_for([]{unsigned mask=0;for(const auto&r:observations)if(r.observation.components==adas::ObjectObservation::All)mask|=1u<<int(r.view);return mask==15&&!geometry.empty();}));
 scene_only.submit(frame(5,2));assert(wait_for([]{for(const auto&r:observations)if(r.stamp.sequence==2)return true;return false;}));
 scene_only.stop();
 assert(geometry.size()==1&&geometry[0].status==adas::Status::Missing&&geometry[0].components==adas::GeometryObservation::Lane);
 assert(legacy_calls==0&&initializations==2);
 std::cout<<"central NPU worker ownership/generation/views passed\n";
}
