#ifndef ADAS_COLLISION_GEOMETRY_H
#define ADAS_COLLISION_GEOMETRY_H
#include "core/adas_types.h"
#include "core/local_planner.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace adas {
// Image-space contact geometry only; this is not a metric distance estimate.
// Callers must mark stale lane observations Missing before passing them here.
class CollisionGeometry {
public:
  static constexpr float near_top = .85f, near_bottom = 1.f;
  // Normalized points ordered left-bottom, left-top, right-top, right-bottom.
  static std::array<PathPoint,4> near_polygon(const LaneObservation &road = {}) {
    auto top=path_edges(near_top,road),bottom=path_edges(near_bottom,road);
    if(!usable_lane(road)){top={.42f,.58f};bottom={.35f,.65f};}
    return {{{bottom.first,near_bottom},{top.first,near_top},
             {top.second,near_top},{bottom.second,near_bottom}}};
  }
  static bool ground_in_near(const Box &box,int width,int height,
                             const LaneObservation &road = {}) {
    if(!valid_box(box,width,height))return false;
    const float y=box.bottom/height;
    if(y<near_top||y>near_bottom)return false;
    const auto polygon=near_polygon(road);
    const float t=(y-near_top)/(near_bottom-near_top);
    const float left=polygon[1].x+t*(polygon[0].x-polygon[1].x);
    const float right=polygon[2].x+t*(polygon[3].x-polygon[2].x);
    return contact_overlaps(box,width,left,right);
  }
  // The forward path deliberately has no near-zone depth cut. Far measured
  // contacts can contribute distance/TTC without entering the short near zone.
  static bool ground_in_path(const Box &box,int width,int height,
                             const LaneObservation &road = {}) {
    if(!valid_box(box,width,height))return false;
    const auto edges=path_edges(box.bottom/height,road);
    return contact_overlaps(box,width,edges.first,edges.second);
  }
private:
  static bool contact_overlaps(const Box &box,int width,float left,float right) {
    const float center=(box.left+box.right)*.5f/width;
    if(center>=left&&center<=right)return true;
    // A close wide target can cover our path while its center is still in
    // another lane. Significant bottom-edge occupancy is a real obstruction;
    // a shallow edge scrape cannot satisfy this separate check.
    const float contact_left=std::max(0.f,box.left/width);
    const float contact_right=std::min(1.f,box.right/width);
    const float path_center=(left+right)*.5f;
    const float occupied=std::min(contact_right,right)-std::max(contact_left,left);
    if(contact_left<=path_center&&contact_right>=path_center&&
       occupied>=(right-left)*.65f)return true;
    // A cutting-in vehicle may occupy the path before its center reaches it.
    // Use only the central half of its bottom edge, capped at 16% of image
    // width, and require 35% occupancy. A full-box edge scrape cannot qualify.
    const float half=std::min((box.right-box.left)*.25f/width,.08f);
    const float overlap=std::min(center+half,right)-std::max(center-half,left);
    return overlap>=half*.7f;
  }
  static bool valid_box(const Box &box,int width,int height) {
    return width>0&&height>0&&std::isfinite(box.left)&&std::isfinite(box.right)&&
      std::isfinite(box.top)&&std::isfinite(box.bottom)&&box.right>box.left&&
      box.bottom>box.top&&box.bottom>=0&&box.bottom<=height;
  }
  static bool usable_lane(const LaneObservation &road) {
    return LocalPlanner::lane_valid(road)&&road.lane.top_y<=road.height*near_top&&
      road.lane.bottom_y>=road.height*.9f;
  }
  static std::pair<float,float> path_edges(float y,const LaneObservation &road) {
    if(!usable_lane(road))return {.35f,.65f};
    const auto &l=road.lane;
    const float t=std::clamp((y*road.height-l.top_y)/(l.bottom_y-l.top_y),0.f,1.f);
    const float left=(l.left_top_x+t*(l.left_x-l.left_top_x))/road.width;
    const float right=(l.right_top_x+t*(l.right_x-l.right_top_x))/road.width;
    const float center=(left+right)*.5f,half=std::min((right-left)*.4f,.15f);
    return {std::clamp(center-half,0.f,1.f),std::clamp(center+half,0.f,1.f)};
  }
};
} // namespace adas
#endif
