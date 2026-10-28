#ifndef ADAS_NEURAL_SCENE_ADAPTER_H
#define ADAS_NEURAL_SCENE_ADAPTER_H
#include "common/frame_packet.h"
#include "common/vision_result.h"
#include "inference/neural_contract.h"
#include <algorithm>

namespace adas {
// Adapt network outputs only. No source pixels or FPGA operations are involved.
inline void adapt_neural_detections(const std::vector<NeuralDetection> &found,
    const NeuralModelSpec &spec,SceneModelRole role,const ViewRect &crop,
    const ViewRect &dst,const std::array<Roi,3> &lights,VisionFrameResult &v) {
  auto &o=v.observation;
  if(o.width<=0||o.height<=0||dst.w<=0||dst.h<=0)return;
  const auto inside=[](double x,double y,const Roi &r){return x>=r.x&&y>=r.y&&x<r.x+r.w&&y<r.y+r.h;};
  const auto priority=[](LightState s){return s==LightState::Red?4:s==LightState::Yellow?3:s==LightState::Green?2:s==LightState::Black?1:0;};
  if(role!=SceneModelRole::Vehicle){o.has_lane_lights=true;o.lane_lights.fill(LightState::Unknown);}
  for(const auto &d:found) {
    if(d.class_id<0||d.class_id>=int(spec.classes.size()))continue;
    const auto &name=spec.classes[d.class_id];
    const bool vehicle=name=="car"||name=="bus"||name=="truck"||name=="motorcycle"||name=="bicycle";
    if(role==SceneModelRole::Vehicle&&name!="person"&&!vehicle)continue;
    Box box{float(crop.x+(d.box.left-dst.x)*double(crop.w)/dst.w),float(crop.y+(d.box.top-dst.y)*double(crop.h)/dst.h),
      float(crop.x+(d.box.right-dst.x)*double(crop.w)/dst.w),float(crop.y+(d.box.bottom-dst.y)*double(crop.h)/dst.h)};
    box.left=std::clamp(box.left,float(crop.x),float(crop.x+crop.w));box.right=std::clamp(box.right,float(crop.x),float(crop.x+crop.w));
    box.top=std::clamp(box.top,float(crop.y),float(crop.y+crop.h));box.bottom=std::clamp(box.bottom,float(crop.y),float(crop.y+crop.h));
    if(box.right-box.left<=1||box.bottom-box.top<=1)continue;
    const double cx=(box.left+box.right)/(2*o.width),cy=(box.top+box.bottom)/(2*o.height);
    const int kind=name=="person"?(role==SceneModelRole::Traffic?12:1):vehicle?10:-1;
    if(role==SceneModelRole::Vehicle&&(kind==1||kind==10))o.obstacles.push_back({kind,-1,box,d.confidence,-1});
    if(role!=SceneModelRole::Vehicle&&name=="crosswalk")o.has_zebra=true;
    const auto lamp=[&](const std::string &base){return name==base||name==base+"_left"||name==base+"_straight"||name==base+"_right";};
    if(role!=SceneModelRole::Vehicle) {
      const auto light=lamp("light_red")?LightState::Red:lamp("light_yellow")?LightState::Yellow:
        lamp("light_green")?LightState::Green:lamp("light_black")?LightState::Black:LightState::Unknown;
      if(priority(light)>priority(o.light))o.light=light;
      const int lane=name.find("_left")!=std::string::npos?0:name.find("_straight")!=std::string::npos?1:name.find("_right")!=std::string::npos?2:-1;
      for(int i=0;i<3;++i)if((lane<0||lane==i)&&inside(cx,cy,lights[i])&&priority(light)>priority(o.lane_lights[i]))o.lane_lights[i]=light;
    }
    OverlayBoxResult overlay;overlay.left=int(box.left);overlay.top=int(box.top);overlay.right=int(box.right);overlay.bottom=int(box.bottom);
    overlay.class_id=kind;overlay.confidence=d.confidence;overlay.label=name;overlay.color_r=kind==1||kind==11||kind==12?255:0;overlay.color_b=kind==10?255:0;
    v.overlays.push_back(std::move(overlay));
  }
  if(role!=SceneModelRole::Vehicle)for(const auto &person:v.overlays)if(person.class_id==1||person.class_id==12)
    for(const auto &z:v.overlays)if(z.label=="crosswalk"&&person.right>z.left&&person.left<z.right&&person.bottom>=z.top&&person.bottom<=z.bottom)o.person_on_zebra=true;
}
}
#endif
