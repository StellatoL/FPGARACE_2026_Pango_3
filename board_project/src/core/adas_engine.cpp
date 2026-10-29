#include "core/adas_engine.h"
#include "core/collision_geometry.h"
#include "core/local_planner.h"
#include "core/tracking_evidence.h"
#include <algorithm>
#include <cmath>

namespace adas {
void AdasEngine::reset(uint64_t generation) {
  if(generation<generation_)return;
  // Keep operator-adjustable thresholds across source changes.
  const double front = front_timeout_s, side = side_timeout_s, speed = external_speed_timeout_s;
  const float ttc = ttc_threshold_s;
  const unsigned components = required_obstacle_components;
  const bool separate_crosswalk = separate_crosswalk_source;
  *this = AdasEngine(); generation_ = generation;
  front_timeout_s = front; side_timeout_s = side; ttc_threshold_s = ttc;
  external_speed_timeout_s = speed;
  required_obstacle_components = components;
  separate_crosswalk_source = separate_crosswalk;
}
bool AdasEngine::accept(const Stamp &s, const Stamp &old) {
  return s.generation == generation_ && s.sequence > old.sequence &&
         std::isfinite(s.source_s) && std::isfinite(s.receive_s) && s.source_s>=0 && s.receive_s>=0;
}
bool AdasEngine::fresh(const Stamp &s, double now, double timeout) {
  return s.sequence && std::isfinite(now) && now + 1e-6 >= s.receive_s &&
         (s.static_image || now-s.receive_s <= timeout);
}
bool AdasEngine::obstacle_sources_ready(int view,double now,double request_s) const {
  if(required_obstacle_components!=ObjectObservation::All&&
     required_obstacle_components!=ObjectObservation::Combined)return false;
  const auto &p=objects_[view];
  if(p.width<=0||p.height<=0)return false;
  const double timeout=view==0?front_timeout_s:side_timeout_s;
  const unsigned components[]={ObjectObservation::Traffic,ObjectObservation::Vehicle,ObjectObservation::Roadblock};
  const Status statuses[]={p.traffic_status,p.vehicle_status,p.roadblock_status};
  const Stamp stamps[]={traffic_stamps_[view],vehicle_stamps_[view],roadblock_stamps_[view]};
  for(int kind=0;kind<3;++kind)if(required_obstacle_components&components[kind]) {
    if(statuses[kind]!=Status::Valid||!fresh(stamps[kind],now,timeout)||
       (request_s>0&&stamps[kind].receive_s<=request_s))return false;
  }
  return true;
}
bool AdasEngine::ingest(const GeometryObservation &g,double now_s) {
  if (g.stamp.generation > generation_) reset(g.stamp.generation);
  bool accepted=false;
  if(g.components&GeometryObservation::Lane)
    accepted=ingest(LaneObservation{g.stamp,g.status,g.width,g.height,g.lane,g.process_ms},now_s)||accepted;
  if(g.components&GeometryObservation::Navigation)
    accepted=ingest(NavigationObservation{g.stamp,g.status,g.width,g.height,g.nav},now_s)||accepted;
  // Legacy visual-speed observations remain inspectable by diagnostic callers,
  // but cannot become an external speed sample or refresh lane/navigation.
  if((g.components&GeometryObservation::Speed)&&accept(g.stamp,legacy_speed_stamp_)) {
    legacy_speed_stamp_=g.stamp;accepted=true;
  }
  return accepted;
}
bool AdasEngine::ingest(const LaneObservation &g,double now_s) {
  if(now_s<0)now_s=g.stamp.receive_s;
  if(g.stamp.generation>generation_)reset(g.stamp.generation);
  if(!accept(g.stamp,lane_.stamp))return false;
  const bool gap=lane_.stamp.sequence&&g.stamp.receive_s-lane_.stamp.receive_s>front_timeout_s;
  lane_=g;
  const bool valid=g.status==Status::Valid&&fresh(g.stamp,now_s,front_timeout_s);
  if(!valid||gap) {
    left_solid_={};left_dashed_={};right_solid_={};right_dashed_={};
    left_solid=left_dashed=right_solid=right_dashed=false;
    if(!valid)return true;
  }
  left_solid=left_solid_.update(valid&&g.lane.left_valid&&g.lane.left_type==LineType::Solid,g.stamp.static_image);
  left_dashed=left_dashed_.update(valid&&g.lane.left_valid&&g.lane.left_type==LineType::Dashed,g.stamp.static_image);
  right_solid=right_solid_.update(valid&&g.lane.right_valid&&g.lane.right_type==LineType::Solid,g.stamp.static_image);
  right_dashed=right_dashed_.update(valid&&g.lane.right_valid&&g.lane.right_type==LineType::Dashed,g.stamp.static_image);
  return true;
}
bool AdasEngine::ingest(const NavigationObservation &g,double now_s) {
  if(now_s<0)now_s=g.stamp.receive_s;
  if(g.stamp.generation>generation_)reset(g.stamp.generation);
  if(!accept(g.stamp,navigation_.stamp))return false;
  const bool gap=navigation_.stamp.sequence&&g.stamp.receive_s-navigation_.stamp.receive_s>front_timeout_s;
  navigation_=g;
  const bool valid=g.status==Status::Valid&&fresh(g.stamp,now_s,front_timeout_s);
  if(!valid||gap) {
    nav_confirm_={};confirmed_nav_=Direction::None;
    if(!valid)return true;
  }
  confirmed_nav_=Direction::None;
  for(int i=0;i<3;++i) {
    const auto direction=static_cast<Direction>(i+1);
    const bool raw=valid&&g.nav.detected&&g.nav.confidence>=.5f&&g.nav.dir==direction;
    if(nav_confirm_[i].update(raw,g.stamp.static_image)&&raw)confirmed_nav_=direction;
  }
  return true;
}
bool AdasEngine::ingest(const SpeedSample &sample,double) {
  if(sample.stamp.generation>generation_)reset(sample.stamp.generation);
  if(!accept(sample.stamp,speed_.stamp))return false;
  speed_=sample;return true;
}
bool AdasEngine::ingest(const SignalObservation &p,double now_s) {
  if(now_s<0)now_s=p.stamp.receive_s;
  if(p.stamp.generation>generation_)reset(p.stamp.generation);
  if(!accept(p.stamp,signal_stamp_))return false;
  const bool gap=signal_stamp_.sequence&&p.stamp.receive_s-signal_stamp_.receive_s>front_timeout_s;
  signal_stamp_=p.stamp;signal_status_=p.status;
  const bool valid=p.status==Status::Valid&&fresh(p.stamp,now_s,front_timeout_s);
  if(!valid||gap) {
    light_confirm_={};green_ready_={};lights_={};
    // Failure is not an empty scene: hide old labels while preserving the
    // safety context until a fresh valid observation describes the junction.
    if(!valid)return true;
  }
  const bool lane_signal_context=p.has_lane_lights&&std::any_of(p.lane_lights.begin(),p.lane_lights.end(),
    [](LightState light){return light!=LightState::Unknown;});
  for(int lane=0;lane<3;++lane) {
    const auto raw=p.has_lane_lights?p.lane_lights[lane]:p.light;
    const bool no_signal=raw==LightState::Unknown||raw==LightState::Black;
    if(no_signal) {
      lights_[lane]=LightState::Unknown;light_confirm_[lane]={};green_ready_[lane]=false;
      // A valid fully empty frame removes an implicit junction. A black lamp
      // or an unobserved lane beside detected lamps still has no permission.
      junction_context_[lane]=raw==LightState::Black||lane_signal_context;
      continue;
    }
    junction_context_[lane]=true;
    if(raw!=LightState::Green){green_ready_[lane]=false;light_confirm_[lane][2]={};}
    if(raw!=lights_[lane])lights_[lane]=LightState::Unknown;
    const bool red=light_confirm_[lane][0].update(raw==LightState::Red,p.stamp.static_image);
    const bool yellow=light_confirm_[lane][1].update(raw==LightState::Yellow,p.stamp.static_image);
    const bool green=light_confirm_[lane][2].update(raw==LightState::Green,p.stamp.static_image);
    if(red&&raw==LightState::Red)lights_[lane]=LightState::Red;
    else if(yellow&&raw==LightState::Yellow)lights_[lane]=LightState::Yellow;
    else if(green&&raw==LightState::Green){lights_[lane]=LightState::Green;green_ready_[lane]=true;}
  }
  return true;
}
void AdasEngine::update_risk(const ObjectObservation &p,unsigned component,double now_s) {
  const int kind=component==ObjectObservation::Traffic?0:component==ObjectObservation::Vehicle?1:2;
  auto &risk=risks_[kind];
  risk.nearest=-1; risk.ttc=-1;
  const Status status=kind==0?p.traffic_status:kind==1?p.vehicle_status:p.roadblock_status;
  if(status!=Status::Valid||p.width<=0||p.height<=0)return;
  auto road=lane_;
  if(!fresh(road.stamp,now_s,front_timeout_s))road.status=Status::Missing;
  bool collision=false,predicted=false;
  for(const auto &ob:p.obstacles) {
    if(ob.class_id!=(kind==0?1:kind==1?10:11))continue;
    if(ob.predicted){predicted=true;continue;}
    const bool corridor=CollisionGeometry::ground_in_path(ob.box,p.width,p.height,road);
    const bool near=CollisionGeometry::ground_in_near(ob.box,p.width,p.height,road);
    if(near)collision=true;
    if(!corridor)continue;
    if(ob.distance_m>=0&&(risk.nearest<0||ob.distance_m<risk.nearest))risk.nearest=ob.distance_m;
    if(kind==2&&ob.distance_m>=0&&ob.distance_m<3)collision=true;
    if(ob.track_id<0||p.stamp.static_image)continue;
    auto &h=history_[{ob.class_id,ob.track_id}];
    if(!h.empty()&&p.stamp.source_s<=h.back().t)h.clear();
    h.push_back({p.stamp.source_s,ob.box.bottom-ob.box.top,ob.distance_m});
    while(h.size()>8||(!h.empty()&&p.stamp.source_s-h.front().t>4))h.pop_front();
    if(h.size()<3)continue;
    const auto &a=h.front(), &b=h.back();const double dt=b.t-a.t;
    float ttc=-1;
    if(dt>.05) {
      if(a.z>=0&&b.z>=0&&a.z-b.z>.05f)ttc=b.z/static_cast<float>((a.z-b.z)/dt);
      // Inverse-depth scale gives current TTC, not the start-of-window TTC.
      else if(b.h-a.h>1)ttc=a.h*static_cast<float>(dt)/(b.h-a.h);
    }
    if(ttc>0&&(risk.ttc<0||ttc<risk.ttc))risk.ttc=ttc;
    if(ttc>0&&ttc<ttc_threshold_s)collision=true;
  }
  if(collision){risk.latched=true;risk.clear_count=0;}
  else if(predicted)risk.clear_count=0;
  else if(++risk.clear_count>=3||p.stamp.static_image)risk.latched=false;
}
bool AdasEngine::ingest(const ObjectObservation &sample,double now_s) {
  if(now_s<0)now_s=sample.stamp.receive_s;
  auto p=sample;p.obstacles=usable_obstacles(sample,now_s);
  const int v=static_cast<int>(p.view);
  if(v<0||v>=4)return false;
  if(p.stamp.generation>generation_)reset(p.stamp.generation);
  const bool traffic=(p.components&ObjectObservation::Traffic)&&accept(p.stamp,traffic_stamps_[v]);
  const bool vehicle=(p.components&ObjectObservation::Vehicle)&&accept(p.stamp,vehicle_stamps_[v]);
  const bool roadblock=(p.components&ObjectObservation::Roadblock)&&accept(p.stamp,roadblock_stamps_[v]);
  const bool crosswalk=(p.components&ObjectObservation::Crosswalk)&&accept(p.stamp,crosswalk_stamps_[v]);
  bool signal=false;
  if(v==0&&((p.components&ObjectObservation::Signal)||
     ((p.components&ObjectObservation::Traffic)&&p.signal_observation&&!separate_crosswalk_source))) {
    const auto signal_status=p.signal_status==Status::Missing&&(p.components&ObjectObservation::Traffic)?
      p.traffic_status:p.signal_status;
    signal=ingest(SignalObservation{p.stamp,signal_status,p.light,p.has_lane_lights,p.lane_lights},now_s);
  }
  if(!traffic&&!vehicle&&!roadblock&&!crosswalk)return signal;
  const bool current=fresh(p.stamp,now_s,v==0?front_timeout_s:side_timeout_s);
  if(v==0&&traffic) {
    const bool gap=traffic_stamps_[0].sequence&&p.stamp.receive_s-traffic_stamps_[0].receive_s>front_timeout_s;
    if(!current||p.traffic_status!=Status::Valid||gap) {
      danger_confirm_={};clear_count_=0;
    }
  }
  if(v==0&&crosswalk&&separate_crosswalk_source) {
    const bool gap=crosswalk_stamps_[0].sequence&&
      p.stamp.receive_s-crosswalk_stamps_[0].receive_s>front_timeout_s;
    if(!current||p.crosswalk_status!=Status::Valid||gap) {
      crosswalk_danger_confirm_={};crosswalk_clear_count_=0;
    }
  }
  auto &merged=objects_[v];
  if(traffic||vehicle||roadblock) {
    merged.view=p.view;merged.width=p.width;merged.height=p.height;
    if(p.stamp.sequence>=merged.stamp.sequence)merged.stamp=p.stamp;
  }
  const auto replace_objects=[&](int kind,Status status) {
    auto &items=merged.obstacles;
    items.erase(std::remove_if(items.begin(),items.end(),[&](const Obstacle &o){return o.class_id==kind;}),items.end());
    if(status==Status::Valid)for(const auto &o:p.obstacles)if(o.class_id==kind)items.push_back(o);
  };
  if(traffic) {
    traffic_stamps_[v]=p.stamp;merged.traffic_status=p.traffic_status;
    merged.light=p.light;merged.has_lane_lights=p.has_lane_lights;merged.lane_lights=p.lane_lights;
    if(!separate_crosswalk_source) {
      merged.has_zebra=p.has_zebra;merged.person_on_zebra=p.person_on_zebra;
    }
    replace_objects(1,p.traffic_status);
  }
  if(vehicle) {
    vehicle_stamps_[v]=p.stamp;merged.vehicle_status=p.vehicle_status;
    replace_objects(10,p.vehicle_status);
  }
  if(roadblock) {
    roadblock_stamps_[v]=p.stamp;merged.roadblock_status=p.roadblock_status;
    replace_objects(11,p.roadblock_status);
  }
  if(crosswalk) {
    crosswalk_stamps_[v]=p.stamp;merged.crosswalk_status=p.crosswalk_status;
    if(separate_crosswalk_source) {
      merged.has_zebra=p.has_zebra;merged.person_on_zebra=p.person_on_zebra;
    }
  }
  if(v!=0)return true;
  if(traffic&&current&&p.traffic_status==Status::Valid) {
    bool person_danger=!separate_crosswalk_source&&p.person_on_zebra,immediate=false,predicted=false;
    auto road=lane_;
    if(!fresh(road.stamp,now_s,front_timeout_s))road.status=Status::Missing;
    for(const auto &ob:p.obstacles) {
      if(ob.class_id!=1||p.width<=0||p.height<=0)continue;
      if(ob.predicted){predicted=true;continue;}
      const bool in_path=CollisionGeometry::ground_in_path(ob.box,p.width,p.height,road);
      // Pedestrians retain the earlier braking depth independently of the vehicle near box.
      const bool near=ob.box.bottom>p.height*.75f;
      const bool metric_close=ob.distance_m>=0&&ob.distance_m<3;
      if(in_path&&(near||metric_close)) {
        person_danger=true;immediate=immediate||ob.confidence>=.65f;
      }
    }
    const bool confirmed=(person_danger||!predicted)&&danger_confirm_.update(person_danger,p.stamp.static_image);
    if(immediate||confirmed){emergency_=true;clear_count_=0;}
    else if(predicted)clear_count_=0;
    else if(!person_danger){if(++clear_count_>=3||p.stamp.static_image)emergency_=false;}
    else clear_count_=0;
  }
  if(crosswalk&&separate_crosswalk_source&&current&&p.crosswalk_status==Status::Valid) {
    const bool danger=p.person_on_zebra;
    const bool confirmed=crosswalk_danger_confirm_.update(danger,p.stamp.static_image)&&danger;
    if(confirmed){crosswalk_emergency_=true;crosswalk_clear_count_=0;}
    else if(!danger) {
      if(++crosswalk_clear_count_>=3||p.stamp.static_image)crosswalk_emergency_=false;
    } else crosswalk_clear_count_=0;
  }
  const unsigned components[]={ObjectObservation::Traffic,ObjectObservation::Vehicle,ObjectObservation::Roadblock};
  const bool accepted[]={traffic,vehicle,roadblock};
  const Status statuses[]={p.traffic_status,p.vehicle_status,p.roadblock_status};
  for(int kind=0;kind<3;++kind)if(accepted[kind]) {
    if(current&&statuses[kind]==Status::Valid)update_risk(p,components[kind],now_s);
    else risks_[kind].clear_count=0;
  }
  for(auto it=history_.begin();it!=history_.end();) {
    if(it->second.empty()||p.stamp.source_s-it->second.back().t>4)it=history_.erase(it);else ++it;
  }
  return true;
}
AdasDecision AdasEngine::evaluate(const EgoState &ego,double now) const {
  AdasDecision d;
  const auto &front=objects_[0];
  const auto front_obstacles=usable_obstacles(front,now);
  const bool front_ok=obstacle_sources_ready(0,now);
  const bool geometry_ok=lane_.status==Status::Valid&&fresh(lane_.stamp,now,front_timeout_s);
  const bool navigation_ok=navigation_.status==Status::Valid&&fresh(navigation_.stamp,now,front_timeout_s);
  int lane=std::clamp(static_cast<int>(ego.lane),0,2);
  const bool signal_current=signal_status_==Status::Valid&&fresh(signal_stamp_,now,front_timeout_s);
  d.light=signal_current?lights_[lane]:LightState::Unknown;
  if(d.light==LightState::Green&&(!signal_current||!green_ready_[lane]))d.light=LightState::Unknown;
  if(ego.speed_source==SpeedSource::Simulation) {
    d.speed_valid=std::isfinite(ego.manual_speed_kmh)&&ego.manual_speed_kmh>=0;
    d.speed_kmh=d.speed_valid?ego.manual_speed_kmh:0;
  } else if(ego.speed_source==SpeedSource::External) {
    // Static-image exceptions are intentionally excluded from external speed.
    d.speed_valid=speed_.status==Status::Valid&&speed_.source==SpeedSource::External&&
      speed_.stamp.sequence&&std::isfinite(now)&&now>=speed_.stamp.receive_s&&
      now-speed_.stamp.receive_s<=external_speed_timeout_s&&
      std::isfinite(speed_.speed_kmh)&&speed_.speed_kmh>=0;
    d.speed_kmh=d.speed_valid?speed_.speed_kmh:0;
  }
  d.speed_source=d.speed_valid?ego.speed_source:SpeedSource::Unavailable;
  d.alarm_overspeed=d.speed_valid&&d.speed_kmh>ego.speed_limit+.5f;
  d.brake_now=emergency_||crosswalk_emergency_;
  d.alarm_collision=(risks_[0].latched||risks_[1].latched||risks_[2].latched)&&!ego.manual_stop;
  for(int kind=0;kind<3;++kind) {
    const auto &stamp=kind==0?traffic_stamps_[0]:kind==1?vehicle_stamps_[0]:roadblock_stamps_[0];
    const auto status=kind==0?front.traffic_status:kind==1?front.vehicle_status:front.roadblock_status;
    if(status!=Status::Valid||!fresh(stamp,now,front_timeout_s))continue;
    const auto &risk=risks_[kind];
    if(risk.nearest>=0&&(d.nearest_obstacle_m<0||risk.nearest<d.nearest_obstacle_m))d.nearest_obstacle_m=risk.nearest;
    if(risk.ttc>0&&(d.ttc_s<0||risk.ttc<d.ttc_s))d.ttc_s=risk.ttc;
  }
  const bool known_junction=ego.intersection_mode||junction_context_[lane];
  const bool unknown_signal=known_junction&&d.light==LightState::Unknown;
  const bool signal_stop=d.light==LightState::Red||d.light==LightState::Yellow||
    (known_junction&&(!signal_current||!green_ready_[lane]));
  d.stop_required=!front_ok||ego.manual_stop||ego.paused||signal_stop||d.brake_now||d.alarm_collision;
  d.alarm_red_run=ego.force_drive&&front_ok&&d.light==LightState::Red;
  d.auto_driving=(ego.automatic&&!d.stop_required)||(ego.force_drive&&front_ok&&!unknown_signal&&!ego.manual_stop&&!ego.paused&&!d.brake_now&&!d.alarm_collision);
  const bool junction=known_junction||d.light!=LightState::Unknown;
  d.arrow=junction ? (lane==0?Direction::Left:lane==2?Direction::Right:Direction::Straight) : Direction::Straight;
  Direction nav=ego.manual_nav_enabled ? ego.manual_nav : navigation_ok ? confirmed_nav_ : Direction::None;
  TurnSignal command=ego.force_lane_change!=TurnSignal::None ? ego.force_lane_change :
    junction ? TurnSignal::None : ego.signal;
  if(junction&&command==TurnSignal::None&&nav!=Direction::None)d.arrow=nav;
  if(!junction&&command==TurnSignal::None) {
    if(nav==Direction::Left)command=TurnSignal::Left;
    if(nav==Direction::Right)command=TurnSignal::Right;
  }
  if(command!=TurnSignal::None) {
    const bool left=command==TurnSignal::Left;
    const auto raw=left?lane_.lane.left_type:lane_.lane.right_type;
    const bool boundary_valid=left?lane_.lane.left_valid:lane_.lane.right_valid;
    const bool solid=geometry_ok&&boundary_valid&&raw==LineType::Solid&&(left?left_solid:right_solid);
    const bool dashed=geometry_ok&&boundary_valid&&raw==LineType::Dashed&&(left?left_dashed:right_dashed);
    const int side_index=left?3:1;
    const auto &side=objects_[side_index];
    const bool side_ok=obstacle_sources_ready(side_index,now,ego.lane_change_request_s);
    bool blocked=false;
    for(const auto &ob:usable_obstacles(side,now))if(ob.box.bottom>side.height*.4f&&(ob.confidence>=.25f||ob.predicted))blocked=true;
    const bool boundary=(left&&lane==0)||(!left&&lane==2);
    const bool single_input=ego.input_layout==Layout::Single;
    const bool target_blocked=LocalPlanner::target_path_blocked(lane_,front_obstacles,
      left?Direction::Left:Direction::Right);
    const bool lateral_clear=!target_blocked&&(single_input||(side_ok&&!blocked));
    d.alarm_illegal_lane_change=solid&&ego.force_lane_change!=TurnSignal::None;
    d.lane_change_allowed=front_ok&&dashed&&lateral_clear&&!boundary&&!d.stop_required;
    d.arrow=d.lane_change_allowed?(left?Direction::Left:Direction::Right):Direction::Straight;
    d.hint_text=!front_ok?u8"前视感知不可用":d.stop_required?u8"停车状态：保持车道":
      boundary?u8"已在最外侧车道":solid?u8"实线：保持车道":!dashed?u8"车道信息不足":
      !LocalPlanner::lane_valid(lane_)?u8"车道几何不可用":target_blocked?u8"前视目标路径存在障碍":
      !single_input&&!side_ok?u8"等待目标侧新有效观测":!single_input&&blocked?u8"目标侧存在障碍":u8"允许模拟变道";
  }
  if(geometry_ok&&!ego.manual_stop&&lane_.lane.left_valid&&lane_.lane.right_valid) {
    const float offset=lane_.lane.center_offset;
    d.alarm_lane_departure=std::abs(offset)>.8f&&(offset<0?left_solid:right_solid);
  }
  auto planning_lane=lane_;if(!geometry_ok)planning_lane.status=Status::Missing;
  d.local_plan=LocalPlanner::plan(planning_lane,front_obstacles,d.arrow,d.lane_change_allowed,d.stop_required);
  if(d.lane_change_allowed&&(!d.local_plan.valid||
     (d.local_plan.maneuver!=Maneuver::ChangeLeft&&d.local_plan.maneuver!=Maneuver::ChangeRight))) {
    d.lane_change_allowed=false;d.arrow=Direction::Straight;d.hint_text=d.local_plan.reason;
    if(!d.local_plan.valid)d.local_plan=LocalPlanner::plan(planning_lane,front_obstacles,d.arrow,false,d.stop_required);
  }
  if(d.local_plan.valid&&d.local_plan.maneuver==Maneuver::Stop&&!d.stop_required) {
    d.stop_required=true;d.auto_driving=false;d.lane_change_allowed=false;d.hint_text=d.local_plan.reason;
  }
  if(!d.auto_driving&&d.stop_required)d.arrow=Direction::None;
  d.led_mask=d.arrow==Direction::Left?1:d.arrow==Direction::Right?4:d.arrow==Direction::Straight?2:0;
  if(d.brake_now)d.status_text+=u8"紧急刹车 ";
  if(d.alarm_red_run)d.status_text+=u8"红灯强行通过 ";
  if(d.alarm_illegal_lane_change)d.status_text+=u8"实线变道报警 ";
  if(d.alarm_collision)d.status_text+=u8"碰撞风险 ";
  if(d.alarm_lane_departure)d.status_text+=u8"压实线 ";
  if(d.alarm_overspeed)d.status_text+=u8"超速 ";
  if(!front_ok)d.status_text+=u8"前视感知不可用：停车等待";
  if(unknown_signal)d.status_text+=u8"路口信号不可用：停车等待";
  if(d.status_text.empty())d.status_text=ego.paused?u8"回放暂停":ego.manual_stop?u8"人工停车":signal_stop?u8"灯控停车":d.auto_driving?u8"模拟行驶":u8"已停车";
  return d;
}
} // namespace adas
