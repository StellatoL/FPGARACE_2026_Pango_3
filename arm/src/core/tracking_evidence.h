#ifndef ADAS_TRACKING_EVIDENCE_H
#define ADAS_TRACKING_EVIDENCE_H
#include "core/adas_types.h"
#include <cmath>
namespace adas {
inline double prediction_lifetime(ViewId view){return view==ViewId::Front?.35:.75;}
inline bool usable_obstacle(const Obstacle &object,const Stamp &stamp,ViewId view,double now){
  if(!object.predicted)return true;
  return !stamp.static_image&&object.track_hits>=2&&std::isfinite(now)&&
    std::isfinite(object.last_observed_s)&&object.last_observed_s>=0&&
    object.last_observed_s<=stamp.receive_s&&now>=object.last_observed_s&&
    now-object.last_observed_s<=prediction_lifetime(view);
}
inline std::vector<Obstacle> usable_obstacles(const ObjectObservation &sample,double now){
  std::vector<Obstacle> out;for(auto object:sample.obstacles)
    if(usable_obstacle(object,sample.stamp,sample.view,now)){if(object.predicted)object.distance_m=-1;out.push_back(object);}
  return out;
}
}
#endif
