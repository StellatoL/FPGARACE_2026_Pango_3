#ifndef ADAS_TRACKED_OVERLAYS_H
#define ADAS_TRACKED_OVERLAYS_H
#include "common/vision_result.h"
#include "core/tracking_evidence.h"
#include <cmath>
namespace adas {
inline void sync_tracking_overlays(VisionFrameResult &result,double now){
  for(const auto &object:result.observation.obstacles){
    if(!usable_obstacle(object,result.stamp,result.view,now))continue;
    OverlayBoxResult *chosen=nullptr;
    if(!object.predicted)for(auto &box:result.overlays)
      if(box.class_id==object.class_id&&std::abs(box.left-object.box.left)<=2&&
         std::abs(box.top-object.box.top)<=2&&std::abs(box.right-object.box.right)<=2&&
         std::abs(box.bottom-object.box.bottom)<=2){chosen=&box;break;}
    if(object.predicted){OverlayBoxResult box;box.left=int(object.box.left);box.top=int(object.box.top);
      box.right=int(object.box.right);box.bottom=int(object.box.bottom);box.class_id=object.class_id;
      box.label=object.class_id==1?"person":object.class_id==10?"vehicle":"roadblock";
      box.color_r=255;box.color_g=190;box.color_b=50;result.overlays.push_back(box);chosen=&result.overlays.back();}
    if(chosen){chosen->track_id=object.track_id;chosen->predicted=object.predicted;
      chosen->last_observed_s=object.last_observed_s;chosen->confidence=object.confidence;chosen->track_hits=object.track_hits;}
  }
}
}
#endif
