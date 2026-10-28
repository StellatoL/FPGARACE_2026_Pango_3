#include "inference/neural_scene_adapter.h"
#include <cassert>
#include <iostream>

int main() {
  using namespace adas;
  const ViewRect crop{0,0,1280,720},content{0,140,640,360};
  std::array<Roi,3> lights;lights.fill({0,0,1,.5});
  NeuralModelSpec spec;spec.classes={"person","car","bus","bicycle","motorcycle","truck","zebra","traffic light","train"};
  std::vector<NeuralDetection> detections;
  for(int i=0;i<9;++i)detections.push_back({i,{100.f+30*i,200,120.f+30*i,250},.8f});
  VisionFrameResult v;v.observation.width=1280;v.observation.height=720;
  adapt_neural_detections(detections,spec,SceneModelRole::Vehicle,crop,content,lights,v);
  assert(v.overlays.size()==6&&v.observation.obstacles.size()==6);
  assert(v.observation.obstacles[0].class_id==1);
  for(size_t i=1;i<6;++i)assert(v.observation.obstacles[i].class_id==10);
  assert(!v.observation.has_zebra&&v.observation.light==LightState::Unknown);
  spec.classes={"crosswalk","person","light_red","light_green","light_black","light_yellow"};
  detections={{0,{100,300,400,450},.8f},{1,{200,250,230,400},.8f},{2,{250,180,260,190},.8f}};
  v={};v.observation.width=1280;v.observation.height=720;
  adapt_neural_detections(detections,spec,SceneModelRole::Traffic,crop,content,lights,v);
  assert(v.observation.obstacles.empty()); // No duplicate primary person clock/trajectory.
  assert(v.observation.has_zebra&&v.observation.person_on_zebra);
  assert(v.observation.light==LightState::Red&&v.observation.lane_lights[1]==LightState::Red);
  assert(v.overlays[1].class_id==12&&v.overlays[1].label=="person");
  detections={{4,{250,180,260,190},.8f}};v={};v.observation.width=1280;v.observation.height=720;
  adapt_neural_detections(detections,spec,SceneModelRole::Traffic,crop,content,lights,v);
  assert(v.observation.light==LightState::Black&&v.observation.lane_lights[0]==LightState::Black);
  detections={{1,{100,10,120,20},.8f}};v={};v.observation.width=1280;v.observation.height=720;
  adapt_neural_detections(detections,spec,SceneModelRole::Traffic,crop,content,lights,v);
  assert(v.overlays.empty()); // A pure letterbox prediction has no source pixels.
  const ViewRect offset_crop{400,100,800,450};
  detections={{1,{-100,100,100,200},.8f}};v={};v.observation.width=1920;v.observation.height=1080;
  adapt_neural_detections(detections,spec,SceneModelRole::Traffic,offset_crop,content,lights,v);
  assert(v.overlays.size()==1&&v.observation.obstacles.empty());
  assert(v.overlays[0].left==400&&v.overlays[0].top==100&&
         v.overlays[0].right==525&&v.overlays[0].bottom==175);
  std::cout<<"COCO class scope, independent traffic people, zebra overlap, lamps and crop mapping: PASS\n";
}
