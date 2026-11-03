#ifndef ADAS_LOCAL_PLANNER_H
#define ADAS_LOCAL_PLANNER_H

#include "core/adas_types.h"
#include <algorithm>
#include <cmath>

namespace adas {
// Numeric postprocessing of model output only. No source pixels, image filters,
// or inferred metric coordinates are used by this bounded candidate planner.
class LocalPlanner {
public:
  static bool lane_valid(const LaneObservation &road) {
    const auto &l=road.lane;
    return road.status==Status::Valid&&road.width>0&&road.height>0&&l.left_valid&&l.right_valid&&
      std::isfinite(l.left_x)&&std::isfinite(l.right_x)&&std::isfinite(l.left_top_x)&&
      std::isfinite(l.right_top_x)&&std::isfinite(l.top_y)&&std::isfinite(l.bottom_y)&&
      l.right_x-l.left_x>=road.width*.08f&&l.right_x-l.left_x<=road.width*1.5f&&
      l.right_top_x>l.left_top_x&&l.top_y>=0&&l.bottom_y>l.top_y&&l.bottom_y<=road.height;
  }

  static bool target_path_blocked(const LaneObservation &road,
                                  const std::vector<Obstacle> &objects,Direction direction) {
    if(!lane_valid(road))return true;
    const auto &l=road.lane;
    const float shift=direction==Direction::Left?-1.f:direction==Direction::Right?1.f:0.f;
    for(const auto &ob:objects) {
      if((ob.class_id!=1&&ob.class_id!=10&&ob.class_id!=11)||(ob.confidence<.25f&&!ob.predicted))continue;
      const auto &b=ob.box;
      if(!std::isfinite(b.left)||!std::isfinite(b.right)||!std::isfinite(b.top)||
         !std::isfinite(b.bottom)||b.right<=b.left||b.bottom<=b.top)return true;
      if(b.bottom<road.height*.4f||b.bottom<l.top_y)continue;
      const float y=std::clamp(b.bottom,l.top_y,l.bottom_y);
      const float t=(y-l.top_y)/(l.bottom_y-l.top_y);
      const float left=l.left_top_x+t*(l.left_x-l.left_top_x);
      const float right=l.right_top_x+t*(l.right_x-l.right_top_x);
      const float width=right-left,center=(left+right)*.5f+shift*width;
      if(b.right>center-width*.45f&&b.left<center+width*.45f)return true;
    }
    return false;
  }

  static LocalPlan plan(const LaneObservation &road,const std::vector<Obstacle> &objects,
                        Direction direction,bool change_allowed,bool stop) {
    LocalPlan result;
    if(stop) {result.valid=true;result.reason=u8"停车约束";return result;}
    if(!lane_valid(road)) {result.reason=u8"车道几何不可用";return result;}
    const bool changing=change_allowed&&(direction==Direction::Left||direction==Direction::Right);
    const Direction candidate=changing?direction:Direction::Straight;
    if(target_path_blocked(road,objects,candidate)) {
      result.valid=true;result.reason=u8"候选路径存在障碍";return result;
    }
    const auto &l=road.lane;
    result.maneuver=changing?(direction==Direction::Left?Maneuver::ChangeLeft:Maneuver::ChangeRight):Maneuver::KeepLane;
    const float shift=changing?(direction==Direction::Left?-1.f:1.f):0.f;
    if(changing)for(const auto &ob:objects) {
      if((ob.class_id!=1&&ob.class_id!=10&&ob.class_id!=11)||(ob.confidence<.25f&&!ob.predicted)||ob.box.bottom<l.top_y)continue;
      const float y=std::clamp(ob.box.bottom,l.top_y,l.bottom_y);
      const float t=(l.bottom_y-y)/(l.bottom_y-l.top_y);
      const float left=l.left_x+t*(l.left_top_x-l.left_x);
      const float right=l.right_x+t*(l.right_top_x-l.right_x);
      const float blend=t*t*(3-2*t),width=right-left;
      const float center=(left+right)*.5f+shift*width*blend;
      if(ob.box.right>center-width*.45f&&ob.box.left<center+width*.45f) {
        auto fallback=plan(road,objects,Direction::Straight,false,false);
        fallback.reason=fallback.maneuver==Maneuver::KeepLane?
          u8"变道过渡路径受阻：保持车道":u8"变道及保持车道路径受阻：停车";
        return fallback;
      }
    }
    for(unsigned i=0;i<result.points.size();++i) {
      const float t=float(i)/float(result.points.size()-1);
      const float left=l.left_x+t*(l.left_top_x-l.left_x);
      const float right=l.right_x+t*(l.right_top_x-l.right_x);
      const float blend=t*t*(3-2*t);
      const float x=((left+right)*.5f+shift*(right-left)*blend)/road.width;
      const float y=(l.bottom_y+t*(l.top_y-l.bottom_y))/road.height;
      if(x<0||x>1||y<0||y>1) {
        result={};result.reason=u8"候选路径超出可见道路";return result;
      }
      result.points[i]={x,y};
    }
    result.point_count=static_cast<unsigned>(result.points.size());result.valid=true;
    result.reason=changing?u8"满足约束的模拟变道路径":u8"保持车道路径";
    return result;
  }
};
} // namespace adas
#endif
