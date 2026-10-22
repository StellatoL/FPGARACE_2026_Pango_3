#include "core/adas_engine.h"
#include "core/view_tracker.h"
#include "core/local_planner.h"
#include <cassert>
#include <iostream>

using namespace adas;
void required_capability_and_single_path_safety() {
  AdasEngine engine; EgoState ego; ego.input_layout=Layout::Single;
  ObjectObservation scene;
  scene.stamp={1,1,.1,.1,true};scene.width=640;scene.height=360;
  scene.components=7;scene.traffic_status=scene.vehicle_status=Status::Valid;
  scene.roadblock_status=Status::Missing;
  engine.ingest(scene);
  assert(engine.evaluate(ego,.1).stop_required); // Missing roadblock is not valid empty.
  scene.stamp.sequence=2;scene.roadblock_status=Status::Valid;engine.ingest(scene);
  assert(!engine.evaluate(ego,.1).stop_required);

  GeometryObservation road;road.stamp={1,1,.1,.1,true};road.status=Status::Valid;
  road.width=640;road.height=360;
  road.lane.left_valid=road.lane.right_valid=true;
  road.lane.left_type=road.lane.right_type=LineType::Dashed;
  road.lane.left_x=210;road.lane.right_x=430;
  road.lane.left_top_x=280;road.lane.right_top_x=360;road.lane.top_y=180;road.lane.bottom_y=360;
  engine.ingest(road);ego.signal=TurnSignal::Left;
  auto decision=engine.evaluate(ego,.1);
  assert(decision.lane_change_allowed&&decision.local_plan.valid);
  assert(decision.local_plan.maneuver==Maneuver::ChangeLeft&&decision.local_plan.point_count==16);
  for(const auto &point:decision.local_plan.points)assert(point.x>=0&&point.x<=1&&point.y>=0&&point.y<=1);
  scene.stamp.sequence=3;
  scene.obstacles={{10,1,{40,220,190,330},.9f,-1}};
  engine.ingest(scene);
  assert(!engine.evaluate(ego,.1).lane_change_allowed); // Front-visible target lane occupied.
  assert(!engine.evaluate(ego,.1).alarm_collision); // Outside current driving corridor.
}
void planner_checks_transition_not_only_target_lane() {
  LaneObservation road;road.status=Status::Valid;road.width=640;road.height=360;
  road.lane.left_valid=road.lane.right_valid=true;road.lane.left_x=210;road.lane.right_x=430;
  road.lane.left_top_x=280;road.lane.right_top_x=360;road.lane.top_y=180;road.lane.bottom_y=360;
  // This box misses the final left-lane corridor, but overlaps the transition.
  std::vector<Obstacle> obstacles={{10,1,{270,240,295,290},.9f,-1}};
  assert(!LocalPlanner::target_path_blocked(road,obstacles,Direction::Left));
  const auto result=LocalPlanner::plan(road,obstacles,Direction::Left,true,false);
  assert(result.valid&&result.maneuver==Maneuver::Stop&&result.point_count==0);
  obstacles[0].box={235,240,240,290};
  assert(!LocalPlanner::target_path_blocked(road,obstacles,Direction::Left));
  assert(!LocalPlanner::target_path_blocked(road,obstacles,Direction::Straight));
  const auto fallback=LocalPlanner::plan(road,obstacles,Direction::Left,true,false);
  assert(fallback.valid&&fallback.maneuver==Maneuver::KeepLane&&fallback.point_count==16);
}
void independent_component_contracts() {
  AdasEngine e;EgoState ego;ego.input_layout=Layout::Single;ego.signal=TurnSignal::Left;
  ObjectObservation p;p.stamp={1,3,.3,.3,false};p.components=ObjectObservation::All;
  p.width=640;p.height=360;p.traffic_status=p.vehicle_status=p.roadblock_status=Status::Valid;e.ingest(p);
  for(int i=1;i<=3;++i) {
    GeometryObservation g;g.stamp={1,uint64_t(i),i*.1,i*.1,false};g.components=GeometryObservation::Lane;
    g.status=Status::Valid;g.width=640;g.height=360;
    g.lane.left_valid=g.lane.right_valid=true;g.lane.left_type=g.lane.right_type=LineType::Dashed;
    g.lane.left_x=210;g.lane.right_x=430;g.lane.left_top_x=280;g.lane.right_top_x=360;
    g.lane.top_y=180;g.lane.bottom_y=360;assert(e.ingest(g));
  }
  assert(e.evaluate(ego,.3).lane_change_allowed);
  GeometryObservation nav;nav.stamp={1,8,.4,.4,false};nav.components=GeometryObservation::Navigation;
  nav.status=Status::Failed;assert(e.ingest(nav));
  assert(e.evaluate(ego,.4).lane_change_allowed); // Nav failure cannot wipe lane confirmation.
  LaneObservation road;road.stamp={1,2,.2,.2,false};road.status=Status::Failed;
  assert(!e.ingest(road)); // Lane ordering is checked against lane, not nav.
  road.stamp={1,4,.5,.5,false};assert(e.ingest(road));
  assert(!e.evaluate(ego,.5).lane_change_allowed);

  e.reset(1);ego.signal=TurnSignal::None;ego.intersection_mode=true;
  p.stamp={1,3,.3,.3,false};p.signal_observation=false;e.ingest(p);
  for(int i=1;i<=3;++i) {
    SignalObservation light;light.stamp={1,uint64_t(i),i*.1,i*.1,false};light.status=Status::Valid;light.light=LightState::Green;assert(e.ingest(light));
    NavigationObservation n;n.stamp={1,uint64_t(i),i*.1,i*.1,false};n.status=Status::Valid;
    n.nav={true,Direction::Right,.9f,{}};assert(e.ingest(n));
  }
  assert(e.evaluate(ego,.3).arrow==Direction::Right);
  road.stamp={1,10,.4,.4,false};assert(e.ingest(road));
  assert(e.evaluate(ego,.4).arrow==Direction::Right); // Lane failure cannot rewind navigation.
  assert(e.ingest(nav)); // Navigation failure updates only its own component.
  assert(e.evaluate(ego,.4).arrow==Direction::Straight);
}
void source_request_and_speed_contracts() {
  AdasEngine e;EgoState ego;ego.signal=TurnSignal::Left;ego.lane_change_request_s=.25;
  auto scene=[](int seq,ViewId view) {
    ObjectObservation p;p.stamp={1,uint64_t(seq),seq*.1,seq*.1,false};p.view=view;
    p.width=640;p.height=360;p.components=ObjectObservation::All;
    p.traffic_status=p.vehicle_status=p.roadblock_status=Status::Valid;return p;
  };
  for(int i=1;i<=3;++i) {
    e.ingest(scene(i,ViewId::Front));
    LaneObservation l;l.stamp={1,uint64_t(i),i*.1,i*.1,false};l.status=Status::Valid;
    l.width=640;l.height=360;l.lane.left_valid=l.lane.right_valid=true;
    l.lane.left_type=l.lane.right_type=LineType::Dashed;
    l.lane.left_x=210;l.lane.right_x=430;l.lane.left_top_x=280;l.lane.right_top_x=360;
    l.lane.top_y=180;l.lane.bottom_y=360;e.ingest(l);
  }
  e.ingest(scene(2,ViewId::Left));assert(!e.evaluate(ego,.3).lane_change_allowed);
  auto side=scene(3,ViewId::Left);side.components=ObjectObservation::Combined;e.ingest(side);
  assert(!e.evaluate(ego,.3).lane_change_allowed); // Roadblock component predates request.
  side.components=ObjectObservation::Roadblock;e.ingest(side);
  assert(e.evaluate(ego,.3).lane_change_allowed);
  side=scene(4,ViewId::Left);side.roadblock_status=Status::Missing;e.ingest(side);
  assert(!e.evaluate(ego,.4).lane_change_allowed);
  ego.input_layout=Layout::Single;assert(e.evaluate(ego,.4).lane_change_allowed);
  ego.input_layout=Layout::Mosaic;assert(!e.evaluate(ego,.4).lane_change_allowed);

  ego={};auto d=e.evaluate(ego,.3);
  assert(d.speed_valid&&d.speed_kmh==0&&d.speed_source==SpeedSource::Simulation);
  ego.manual_speed_kmh=80;assert(e.evaluate(ego,.3).alarm_overspeed);
  ego.speed_source=SpeedSource::External;
  assert(!e.evaluate(ego,.3).speed_valid&&!e.evaluate(ego,.3).alarm_overspeed);
  SpeedSample s;s.stamp={1,1,.3,.3,false};s.status=Status::Valid;s.source=SpeedSource::External;s.speed_kmh=80;
  assert(e.ingest(s));assert(e.evaluate(ego,.3).alarm_overspeed);
  assert(!e.evaluate(ego,1.31).speed_valid); // One-second expiry, no simulation fallback.
  s.stamp={0,99,.5,.5,false};assert(!e.ingest(s));assert(e.generation()==1);
  s.stamp={2,1,.5,.5,false};assert(e.ingest(s));assert(e.generation()==2);
  e.reset(1);assert(e.generation()==2); // Explicit stale resets cannot move epochs backwards.
}
void roadblock_tracker_contracts() {
  ViewTracker tracker;ObjectObservation p;
  p.stamp={1,1,.1,.1,false};p.components=ObjectObservation::All;
  p.obstacles={{1,-1,{10,10,60,60},.9f,-1},{10,-1,{10,10,60,60},.9f,-1},{11,-1,{10,10,60,60},.9f,-1}};
  assert(tracker.update(p));assert(p.obstacles.size()==3);
  const auto ids=p.obstacles;
  assert(ids[0].track_id!=ids[1].track_id&&ids[1].track_id!=ids[2].track_id);
  p.stamp.sequence=2;assert(tracker.update(p));
  for(size_t i=0;i<ids.size();++i)assert(p.obstacles[i].track_id==ids[i].track_id);
  p.view=ViewId::Left;assert(tracker.update(p));
  for(size_t i=0;i<ids.size();++i)assert(p.obstacles[i].track_id!=ids[i].track_id);
  p.stamp.generation=0;p.stamp.sequence=99;assert(!tracker.update(p));
}
void isolated_signal_does_not_clear_objects() {
  AdasEngine engine;EgoState ego;
  ObjectObservation p;p.components=ObjectObservation::All;p.stamp={1,1,.1,.1,false};
  p.width=640;p.height=360;p.traffic_status=p.vehicle_status=p.roadblock_status=Status::Valid;
  p.obstacles={{1,1,{280,180,360,350},.9f,-1}};
  engine.ingest(p);assert(engine.evaluate(ego,.1).brake_now);
  for(int i=2;i<=4;++i) {
    ObjectObservation roi;roi.components=ObjectObservation::Signal;roi.stamp={1,uint64_t(i),i*.1,i*.1,false};
    roi.signal_status=Status::Valid;roi.light=LightState::Green;assert(engine.ingest(roi));
  }
  assert(engine.evaluate(ego,.4).brake_now);
  assert(engine.evaluate(ego,.4).light==LightState::Green);
  assert(engine.evaluate(ego,.7).stop_required); // ROI signals cannot refresh missing full-scene detections.
  p.stamp={1,4,.4,.4,false};p.light=LightState::Red;p.obstacles.clear();
  assert(engine.ingest(p)); // Same frame: signal duplicate, but people are a new component.
  assert(engine.evaluate(ego,.4).brake_now);
  p.stamp={1,5,.5,.5,false};engine.ingest(p);
  p.stamp={1,6,.6,.6,false};engine.ingest(p);
  assert(!engine.evaluate(ego,.6).brake_now);
  assert(engine.evaluate(ego,.6).light==LightState::Green); // Duplicate red4 contributed no vote.
  p.stamp={1,7,.7,.7,false};engine.ingest(p);
  assert(engine.evaluate(ego,.7).light==LightState::Red);
}
void roi_signal_authority_ignores_full_scene_absence() {
  AdasEngine engine;EgoState ego;
  for(int i=1;i<=3;++i) {
    for(int j=0;j<3;++j) {
      ObjectObservation full;full.components=ObjectObservation::All;full.signal_observation=false;
      full.stamp={1,uint64_t(i*4+j),i*.1+j*.001,i*.1+j*.001,false};full.width=640;full.height=360;
      full.traffic_status=full.vehicle_status=full.roadblock_status=Status::Valid;engine.ingest(full);
    }
    ObjectObservation roi;roi.components=ObjectObservation::Signal;roi.stamp={1,uint64_t(i*4+3),i*.1+.01,i*.1+.01,false};
    roi.signal_status=Status::Valid;roi.light=LightState::Green;assert(engine.ingest(roi));
  }
  assert(engine.evaluate(ego,.31).light==LightState::Green);
  assert(!engine.evaluate(ego,.31).stop_required);
}
void controlled_junction_waits_for_fresh_confirmed_green() {
  AdasEngine engine;EgoState ego;
  auto front=[&](uint64_t seq,double time) {
    ObjectObservation p;p.components=ObjectObservation::All;p.signal_observation=false;
    p.stamp={1,seq,time,time,false};p.width=640;p.height=360;
    p.traffic_status=p.vehicle_status=p.roadblock_status=Status::Valid;assert(engine.ingest(p));
  };
  auto signal=[&](uint64_t seq,double time,LightState light,Status status=Status::Valid) {
    SignalObservation p;p.stamp={1,seq,time,time,false};p.status=status;p.light=light;assert(engine.ingest(p));
  };
  for(int i=1;i<=3;++i){front(i,i*.1);signal(i,i*.1,LightState::Green);}
  assert(!engine.evaluate(ego,.3).stop_required);
  front(100,.81);
  auto d=engine.evaluate(ego,.81);assert(d.stop_required&&!d.auto_driving&&d.light!=LightState::Green);
  ego.force_drive=true;assert(!engine.evaluate(ego,.81).auto_driving);ego.force_drive=false;
  signal(100,.82,LightState::Unknown,Status::Failed);front(101,.82);
  // Valid absence cannot reuse a green whose authority was lost to an outage.
  for(int i=101;i<=103;++i)signal(i,.9+(i-101)*.1,LightState::Unknown);
  front(104,1.1);assert(engine.evaluate(ego,1.1).stop_required);
  signal(104,1.2,LightState::Green);assert(engine.evaluate(ego,1.2).stop_required);
  signal(105,1.3,LightState::Green);assert(engine.evaluate(ego,1.3).stop_required);
  signal(106,1.4,LightState::Green);assert(!engine.evaluate(ego,1.4).stop_required);
  for(int i=107;i<=109;++i)signal(i,1.5+(i-107)*.1,LightState::Unknown);
  front(109,1.7);assert(!engine.evaluate(ego,1.7).stop_required); // Confirmed departure after fresh green.
  engine.reset(1);front(1,.1);ego.intersection_mode=true;
  assert(engine.evaluate(ego,.1).stop_required); // Explicit intersection, unobserved lamp.
  ego.intersection_mode=false;
  for(int i=1;i<=3;++i)signal(i,i*.1,LightState::Red);
  for(int i=4;i<=6;++i)signal(i,i*.1,LightState::Unknown);
  front(6,.6);assert(engine.evaluate(ego,.6).stop_required);
  assert(engine.evaluate(ego,.6).light==LightState::Red); // Red cannot depart on absence.
}
ObjectObservation objects(int seq, LightState light = LightState::Unknown, ViewId view = ViewId::Front) {
  ObjectObservation p; p.stamp = {1, static_cast<uint64_t>(seq), seq * .1, seq * .1, false};
  p.view = view; p.width = 640; p.height = 360;
  p.components=ObjectObservation::All;
  p.traffic_status = p.vehicle_status = p.roadblock_status = Status::Valid; p.light = light; return p;
}
GeometryObservation geometry(int seq, LineType left = LineType::Dashed) {
  GeometryObservation g; g.stamp = {1, static_cast<uint64_t>(seq), seq * .1, seq * .1, false};
  g.status = Status::Valid; g.width = 640; g.height = 360;
  g.lane.left_valid = g.lane.right_valid = true;
  g.lane.left_type = left; g.lane.right_type = LineType::Dashed;
  g.lane.left_x=210;g.lane.right_x=430;g.lane.left_top_x=280;g.lane.right_top_x=360;
  g.lane.top_y=180;g.lane.bottom_y=360;
  return g;
}
void partial_component_regressions() {
  AdasEngine e;EgoState ego;
  auto sample=[](int seq,unsigned components,ViewId view=ViewId::Front) {
    auto p=objects(seq,LightState::Unknown,view);p.components=components;return p;
  };
  // Sequence ordering is independent for the two models, including a Combined
  // result whose Traffic half has already arrived.
  assert(e.ingest(sample(8,ObjectObservation::Traffic)));
  assert(e.ingest(sample(7,ObjectObservation::Vehicle)));
  assert(!e.ingest(sample(6,ObjectObservation::Vehicle)));
  auto same_frame=sample(8,ObjectObservation::Combined);
  same_frame.obstacles.push_back({1,7,{280,150,360,350},.9f,-1});
  assert(e.ingest(same_frame));
  assert(!e.evaluate(ego,.8).brake_now); // Duplicate Traffic data must not run again.
  assert(!e.ingest(same_frame));

  e.reset(1);ego.signal=TurnSignal::Left;
  for(int i=1;i<=3;++i){e.ingest(sample(i,ObjectObservation::Traffic));e.ingest(geometry(i));}
  e.ingest(sample(3,ObjectObservation::Vehicle|ObjectObservation::Roadblock));
  e.ingest(sample(3,ObjectObservation::Traffic,ViewId::Left));
  e.ingest(sample(3,ObjectObservation::Vehicle,ViewId::Left));
  e.ingest(sample(3,ObjectObservation::Roadblock,ViewId::Left));
  assert(e.evaluate(ego,.3).lane_change_allowed);
  e.ingest(objects(20));
  for(int i=18;i<=20;++i)e.ingest(geometry(i)); // Reconfirm geometry after the gap.
  e.ingest(sample(20,ObjectObservation::Traffic,ViewId::Left));
  e.ingest(sample(20,ObjectObservation::Roadblock,ViewId::Left));
  assert(!e.evaluate(ego,2).lane_change_allowed); // Vehicle is still from .3s.
  assert(!e.ingest(sample(2,ObjectObservation::Vehicle,ViewId::Left)));
  assert(!e.evaluate(ego,2).lane_change_allowed);
  e.ingest(sample(21,ObjectObservation::Vehicle,ViewId::Left));
  assert(e.evaluate(ego,2.1).lane_change_allowed);
  e.ingest(objects(40));
  for(int i=38;i<=40;++i)e.ingest(geometry(i));
  e.ingest(sample(40,ObjectObservation::Vehicle,ViewId::Left));
  e.ingest(sample(40,ObjectObservation::Roadblock,ViewId::Left));
  assert(!e.evaluate(ego,4).lane_change_allowed); // New Vehicle cannot refresh old Traffic.
  e.ingest(sample(41,ObjectObservation::Traffic,ViewId::Left));
  assert(e.evaluate(ego,4.1).lane_change_allowed);

  e.reset(1);
  auto still=sample(1,ObjectObservation::Traffic);still.stamp.static_image=true;e.ingest(still);
  auto extra=still;extra.components=ObjectObservation::Vehicle|ObjectObservation::Roadblock;e.ingest(extra);
  auto still_geometry=geometry(1);still_geometry.stamp.static_image=true;e.ingest(still_geometry);
  still.view=ViewId::Left;e.ingest(still);
  extra.view=ViewId::Left;extra.components=ObjectObservation::Roadblock;e.ingest(extra);
  assert(!e.evaluate(ego,100).lane_change_allowed);
  still.components=ObjectObservation::Vehicle;e.ingest(still);
  assert(e.evaluate(ego,100).lane_change_allowed); // Direct static confirmation, independent components.

  e.reset(1);ego={};e.ingest(sample(10,ObjectObservation::Traffic));
  auto hazard=sample(10,ObjectObservation::Vehicle);
  hazard.obstacles.push_back({10,42,{280,200,360,350},.9f,-1});
  e.ingest(hazard);assert(e.evaluate(ego,1).alarm_collision);
  for(int i=11;i<=20;++i)e.ingest(sample(i,ObjectObservation::Traffic));
  assert(e.evaluate(ego,2).alarm_collision); // Traffic cannot clear vehicle risk.
  auto failed=sample(21,ObjectObservation::Vehicle);failed.vehicle_status=Status::Failed;
  e.ingest(failed);assert(e.evaluate(ego,2.1).alarm_collision);
  e.ingest(sample(22,ObjectObservation::Vehicle));e.ingest(sample(23,ObjectObservation::Vehicle));
  assert(e.evaluate(ego,2.3).alarm_collision);
  assert(!e.ingest(sample(20,ObjectObservation::Vehicle)));
  e.ingest(sample(24,ObjectObservation::Vehicle));
  assert(!e.evaluate(ego,2.4).alarm_collision);

  hazard=sample(30,ObjectObservation::Traffic);
  hazard.obstacles.push_back({1,7,{280,180,360,290},.9f,-1});
  e.ingest(hazard);assert(e.evaluate(ego,3).brake_now);
  for(int i=31;i<=33;++i)e.ingest(sample(i,ObjectObservation::Vehicle));
  assert(e.evaluate(ego,3.3).brake_now);
  failed=sample(34,ObjectObservation::Traffic);failed.traffic_status=Status::Failed;
  e.ingest(failed);assert(e.evaluate(ego,3.4).brake_now);
  for(int i=35;i<=37;++i)e.ingest(sample(i,ObjectObservation::Traffic));
  assert(!e.evaluate(ego,3.7).brake_now);

  e.reset(1);auto unknown=sample(1,ObjectObservation::Vehicle);
  unknown.obstacles.push_back({99,42,{280,200,360,350},.9f,-1});
  e.ingest(unknown);
  assert(!e.evaluate(ego,.1).alarm_collision); // Unsupported classes are not vehicles.
}
void roadblock_adapter_regressions() {
  AdasEngine e;EgoState ego;
  auto block=objects(1);block.components=4;block.roadblock_status=Status::Valid;
  block.obstacles={{11,99,{280,200,360,350},.9f,1.5f}};
  assert(e.ingest(block));
  assert(e.evaluate(ego,.1).alarm_collision);
  for(int i=2;i<=5;++i){auto other=objects(i);other.components=ObjectObservation::Combined;e.ingest(other);}
  assert(e.evaluate(ego,.5).alarm_collision); // Other models cannot clear a roadblock.
  block.stamp={1,6,.6,.6,false};block.roadblock_status=Status::Failed;block.obstacles.clear();
  assert(e.ingest(block));assert(e.evaluate(ego,.6).alarm_collision);
  block.roadblock_status=Status::Valid;
  for(int i=7;i<=9;++i){block.stamp={1,uint64_t(i),i*.1,i*.1,false};assert(e.ingest(block));}
  assert(!e.evaluate(ego,.9).alarm_collision);
  assert(!e.ingest(block));

  e.reset(1);ego.signal=TurnSignal::Left;
  for(int i=1;i<=3;++i){e.ingest(objects(i));e.ingest(geometry(i));}
  auto side=objects(3,LightState::Unknown,ViewId::Left);side.components=ObjectObservation::Combined;e.ingest(side);
  assert(!e.evaluate(ego,.3).lane_change_allowed); // Roadblock capability is mandatory.
  block.view=ViewId::Left;block.stamp={1,3,.3,.3,false};block.roadblock_status=Status::Valid;
  block.obstacles.clear();assert(e.ingest(block));assert(e.evaluate(ego,.3).lane_change_allowed);
  block.stamp={1,4,.4,.4,false};
  block.obstacles={{11,99,{100,160,220,340},.9f,-1}};
  assert(e.ingest(block));assert(!e.evaluate(ego,.4).lane_change_allowed);
  block.obstacles.clear();block.stamp={1,5,.5,.5,false};e.ingest(block);
  assert(e.evaluate(ego,.5).lane_change_allowed);
  e.ingest(objects(20));for(int i=18;i<=20;++i)e.ingest(geometry(i));
  side=objects(20,LightState::Unknown,ViewId::Left);side.components=ObjectObservation::Combined;e.ingest(side);
  assert(!e.evaluate(ego,2.01).lane_change_allowed); // Configured roadblock observations expired.
  block.stamp={1,20,2,2,false};block.roadblock_status=Status::Failed;e.ingest(block);
  assert(!e.evaluate(ego,2).lane_change_allowed);
  block.stamp={1,21,2.1,2.1,false};block.roadblock_status=Status::Valid;e.ingest(block);
  assert(e.evaluate(ego,2.1).lane_change_allowed);
  block.stamp.generation=0;block.stamp.sequence=99;assert(!e.ingest(block));
}
void geometry_timeout_requires_new_confirmation() {
  AdasEngine e;EgoState ego;ego.signal=TurnSignal::Left;
  for(int i=1;i<=3;++i){e.ingest(objects(i));e.ingest(geometry(i));}
  e.ingest(objects(3,LightState::Unknown,ViewId::Left));
  assert(e.evaluate(ego,.3).lane_change_allowed);
  e.ingest(objects(10));e.ingest(objects(10,LightState::Unknown,ViewId::Left));
  e.ingest(geometry(10));
  assert(!e.evaluate(ego,1.).lane_change_allowed); // Old votes expired during the gap.
  e.ingest(geometry(11));assert(!e.evaluate(ego,1.1).lane_change_allowed);
  e.ingest(geometry(12));assert(e.evaluate(ego,1.2).lane_change_allowed);

  e.reset(1);ego.signal=TurnSignal::None;
  for(int i=1;i<=3;++i){
    e.ingest(objects(i));auto g=geometry(i);g.nav={true,Direction::Left,.9f,{}};e.ingest(g);
  }
  e.ingest(objects(3,LightState::Unknown,ViewId::Left));
  assert(e.evaluate(ego,.3).arrow==Direction::Left);
  e.ingest(objects(10));e.ingest(objects(10,LightState::Unknown,ViewId::Left));
  auto g=geometry(10);g.nav={true,Direction::Left,.9f,{}};e.ingest(g);
  assert(e.evaluate(ego,1.).arrow==Direction::Straight);
}
void stopped_decision_does_not_promise_lane_change() {
  AdasEngine e;EgoState ego;ego.signal=TurnSignal::Left;ego.manual_stop=true;
  for(int i=1;i<=3;++i){e.ingest(objects(i));e.ingest(geometry(i));}
  e.ingest(objects(3,LightState::Unknown,ViewId::Left));
  const auto d=e.evaluate(ego,.3);
  assert(!d.lane_change_allowed&&d.arrow==Direction::None);
  assert(d.hint_text!=u8"允许模拟变道");
}
void single_input_rules_regressions() {
  AdasEngine e;EgoState ego;ego.input_layout=Layout::Single;
  auto seed=[&](LineType left,LineType right,bool still=true,LightState light=LightState::Unknown) {
    e.reset(1);
    for(int i=1;i<=(still?1:3);++i) {
      auto p=objects(i,light);p.stamp.static_image=still;e.ingest(p);
      auto g=geometry(i,left);g.lane.right_type=right;g.stamp.static_image=still;e.ingest(g);
    }
  };
  seed(LineType::Dashed,LineType::Dashed);
  assert(e.evaluate(ego,.1).arrow==Direction::Straight); // A request is required.
  for(auto turn:{TurnSignal::Left,TurnSignal::Right}) {
    ego.signal=turn;auto d=e.evaluate(ego,.1);
    assert(d.lane_change_allowed&&d.arrow==(turn==TurnSignal::Left?Direction::Left:Direction::Right));
    assert(d.hint_text==u8"允许模拟变道");
  }
  ego.signal=TurnSignal::Left;
  seed(LineType::Solid,LineType::Solid);
  auto d=e.evaluate(ego,.1);assert(!d.lane_change_allowed&&d.arrow==Direction::Straight&&!d.alarm_illegal_lane_change);
  ego.force_lane_change=TurnSignal::Left;
  d=e.evaluate(ego,.1);assert(d.alarm_illegal_lane_change&&!d.lane_change_allowed&&d.arrow==Direction::Straight);
  ego.force_lane_change=TurnSignal::Right;
  assert(e.evaluate(ego,.1).alarm_illegal_lane_change);
  ego.force_lane_change=TurnSignal::None;
  seed(LineType::Unknown,LineType::Unknown);
  assert(!e.evaluate(ego,.1).lane_change_allowed);
  e.reset(1);auto only_road=geometry(1);only_road.stamp.static_image=true;e.ingest(only_road);
  assert(!e.evaluate(ego,.1).lane_change_allowed); // Missing Traffic is not an empty detection.
  e.reset(1);auto only_traffic=objects(1);only_traffic.stamp.static_image=true;e.ingest(only_traffic);
  assert(!e.evaluate(ego,.1).lane_change_allowed); // Missing Geometry supplies no line type.
  seed(LineType::Dashed,LineType::Dashed);
  ego.lane=LaneId::Left;assert(!e.evaluate(ego,.1).lane_change_allowed);ego.lane=LaneId::Center;
  ego.manual_stop=true;d=e.evaluate(ego,.1);
  assert(!d.lane_change_allowed&&d.arrow==Direction::None&&d.hint_text!=u8"允许模拟变道");
  ego.manual_stop=false;ego.paused=true;assert(!e.evaluate(ego,.1).lane_change_allowed);ego.paused=false;
  seed(LineType::Dashed,LineType::Dashed,true,LightState::Red);
  d=e.evaluate(ego,.1);assert(d.stop_required&&!d.lane_change_allowed&&d.hint_text!=u8"允许模拟变道");
  seed(LineType::Dashed,LineType::Dashed);
  auto failed=objects(2);failed.stamp.static_image=true;failed.traffic_status=Status::Failed;e.ingest(failed);
  d=e.evaluate(ego,.2);assert(!d.lane_change_allowed&&d.hint_text!=u8"允许模拟变道");
  seed(LineType::Dashed,LineType::Dashed);
  auto bad_geometry=geometry(2);bad_geometry.stamp.static_image=true;bad_geometry.status=Status::Failed;e.ingest(bad_geometry);
  assert(!e.evaluate(ego,.2).lane_change_allowed);
  seed(LineType::Dashed,LineType::Dashed);
  auto hazard=objects(2);hazard.stamp.static_image=true;
  hazard.obstacles={{1,1,{280,180,360,350},.95f,-1}};e.ingest(hazard);
  d=e.evaluate(ego,.2);assert(d.brake_now&&!d.lane_change_allowed);
  seed(LineType::Dashed,LineType::Dashed);
  hazard.obstacles={{10,1,{280,180,360,350},.95f,-1}};e.ingest(hazard);
  d=e.evaluate(ego,.2);assert(d.alarm_collision&&!d.lane_change_allowed);

  // Visual navigation must be ingested; no manual-navigation override is used.
  ego.signal=TurnSignal::None;
  for(auto direction:{Direction::Left,Direction::Straight,Direction::Right}) {
    seed(LineType::Dashed,LineType::Dashed);
    auto g=geometry(2);g.stamp.static_image=true;g.nav={true,direction,.9f,{}};e.ingest(g);
    assert(e.evaluate(ego,.2).arrow==direction);
  }
  seed(LineType::Solid,LineType::Solid);
  auto nav=geometry(2,LineType::Solid);nav.lane.right_type=LineType::Solid;
  nav.stamp.static_image=true;nav.nav={true,Direction::Left,.9f,{}};e.ingest(nav);
  d=e.evaluate(ego,.2);assert(!d.lane_change_allowed&&d.arrow==Direction::Straight);

  // Video needs three distinct observations and must reconfirm after a gap.
  seed(LineType::Dashed,LineType::Dashed,false);
  ego.signal=TurnSignal::Left;assert(e.evaluate(ego,.3).lane_change_allowed);
  e.ingest(objects(10));auto g=geometry(10);e.ingest(g);
  for(int i=0;i<10;++i){e.ingest(g);assert(!e.evaluate(ego,1.).lane_change_allowed);}
  e.ingest(geometry(11));assert(!e.evaluate(ego,1.1).lane_change_allowed);
  e.ingest(geometry(12));assert(e.evaluate(ego,1.2).lane_change_allowed);
  assert(!e.evaluate(ego,2).lane_change_allowed);
  seed(LineType::Dashed,LineType::Dashed,false);
  e.ingest(objects(10)); // Fresh Traffic cannot refresh expired Geometry.
  assert(!e.evaluate(ego,1.).lane_change_allowed);
  seed(LineType::Dashed,LineType::Dashed,false);
  for(int i=8;i<=10;++i)e.ingest(geometry(i)); // Geometry cannot refresh Traffic.
  d=e.evaluate(ego,1.);assert(!d.lane_change_allowed&&d.stop_required);

  // The very same scene remains blocked in default/four-view mode.
  seed(LineType::Dashed,LineType::Dashed);
  ego.input_layout=Layout::Mosaic;
  assert(!e.evaluate(ego,.1).lane_change_allowed);
  auto side=objects(1,LightState::Unknown,ViewId::Left);side.stamp.static_image=true;e.ingest(side);
  assert(e.evaluate(ego,.1).lane_change_allowed);
  side.stamp.sequence=2;side.vehicle_status=Status::Failed;e.ingest(side);
  assert(!e.evaluate(ego,.1).lane_change_allowed);
}
int main() {
  required_capability_and_single_path_safety();
  planner_checks_transition_not_only_target_lane();
  independent_component_contracts();
  source_request_and_speed_contracts();
  roadblock_tracker_contracts();
  isolated_signal_does_not_clear_objects();
  roi_signal_authority_ignores_full_scene_absence();
  controlled_junction_waits_for_fresh_confirmed_green();
  single_input_rules_regressions();
  geometry_timeout_requires_new_confirmation();
  stopped_decision_does_not_promise_lane_change();
  roadblock_adapter_regressions();
  partial_component_regressions();
  AdasEngine e; EgoState ego;
  // Delayed observations retain their capability/age but cannot confirm a
  // release of a stop latch. The caller provides its actual receive clock.
  for(int i=1;i<=3;++i)e.ingest(objects(i,LightState::Red));
  for(int i=4;i<=6;++i)e.ingest(objects(i,LightState::Green),2.0);
  e.ingest(objects(20));
  assert(e.evaluate(ego,2.).light==LightState::Red);
  auto close_vehicle=objects(21);close_vehicle.components=ObjectObservation::Vehicle;
  close_vehicle.obstacles={{10,42,{280,200,360,350},.9f,-1}};e.ingest(close_vehicle);
  assert(e.evaluate(ego,2.1).alarm_collision);
  for(int i=22;i<=24;++i){auto clear=objects(i);clear.components=ObjectObservation::Vehicle;e.ingest(clear,4.);}
  assert(e.evaluate(ego,4.).alarm_collision);
  for(int i=40;i<=41;++i){auto clear=objects(i);clear.components=ObjectObservation::Vehicle;e.ingest(clear);}
  assert(e.evaluate(ego,4.1).alarm_collision);
  auto cleared=objects(42);cleared.components=ObjectObservation::Vehicle;e.ingest(cleared);
  assert(!e.evaluate(ego,4.2).alarm_collision);
  e.reset(1);ego.intersection_mode=true;
  for(int i=1;i<=3;++i){auto g=geometry(i);g.nav={true,Direction::Right,.9f,{}};e.ingest(g,2.);}
  auto fresh_nav=geometry(20);fresh_nav.nav={true,Direction::Right,.9f,{}};e.ingest(fresh_nav);
  auto at_junction=objects(20);at_junction.signal_observation=false;e.ingest(at_junction);
  for(int i=18;i<=20;++i) {
    SignalObservation green;green.stamp={1,uint64_t(i),i*.1,i*.1,false};
    green.status=Status::Valid;green.light=LightState::Green;e.ingest(green,2.);
  }
  assert(e.evaluate(ego,2.).arrow==Direction::Straight); // Fresh green; delayed navigation remains unconfirmed.
  e.reset(1);ego={};ego.signal=TurnSignal::Left;
  for(int i=1;i<=3;++i){e.ingest(objects(i));e.ingest(geometry(i));}
  e.ingest(objects(3,LightState::Unknown,ViewId::Left));
  assert(e.evaluate(ego,.3).lane_change_allowed);
  e.ingest(geometry(4,LineType::Unknown));
  assert(!e.evaluate(ego,.4).lane_change_allowed);
  auto invalid_boundary=geometry(5);invalid_boundary.lane.left_valid=false;e.ingest(invalid_boundary);
  assert(!e.evaluate(ego,.5).lane_change_allowed);
  ego={};
  e.reset(1);
  // A one-frame arrow or repeated UI evaluation must not confirm navigation.
  for(int i=1;i<=3;++i){e.ingest(objects(i));e.ingest(geometry(i));}
  e.ingest(objects(3,LightState::Unknown,ViewId::Left));
  auto nav=geometry(4);nav.nav={true,Direction::Left,.9f,{}};e.ingest(nav);
  assert(e.evaluate(ego,.4).arrow==Direction::Straight);
  for(int i=0;i<10;++i){e.ingest(nav);e.evaluate(ego,.4);}
  assert(e.evaluate(ego,.4).arrow==Direction::Straight);
  nav.stamp={1,5,.5,.5,false};e.ingest(nav);
  nav.stamp={1,6,.6,.6,false};e.ingest(nav);
  assert(e.evaluate(ego,.6).arrow==Direction::Left);
  nav.stamp={1,7,.7,.7,false};nav.nav.detected=false;e.ingest(nav);
  assert(e.evaluate(ego,.7).arrow==Direction::Straight);
  e.reset(1);
  ego.lane=LaneId::Left;ego.signal=TurnSignal::Left;
  for(int i=1;i<=3;++i){e.ingest(objects(i,LightState::Green));e.ingest(geometry(i,LineType::Solid));}
  assert(e.evaluate(ego,.3).arrow==Direction::Left); // A junction turn is not a road lane change.
  ego.lane=LaneId::Center;ego.signal=TurnSignal::None;ego.manual_nav_enabled=true;ego.manual_nav=Direction::Left;
  for(int i=4;i<=6;++i){e.ingest(objects(i));e.ingest(geometry(i,LineType::Solid));}
  assert(e.evaluate(ego,.6).light==LightState::Unknown);
  assert(e.evaluate(ego,.6).arrow==Direction::Straight); // Leaving green cannot bypass solid-line/side checks.
  ego={};e.reset(1);
  for(int i=1;i<=3;++i) { e.ingest(objects(i, LightState::Red)); e.ingest(geometry(i)); }
  assert(e.evaluate(ego,.3).stop_required);
  ego.force_drive = true;
  assert(e.evaluate(ego,.3).alarm_red_run);
  ego.force_drive = false;
  e.ingest(objects(4,LightState::Green));
  for(int i=0;i<100;++i) e.evaluate(ego,.4);
  assert(e.evaluate(ego,.4).stop_required); // UI cannot confirm green.
  e.ingest(objects(4,LightState::Green));
  assert(e.evaluate(ego,.4).stop_required); // Repeated source frame cannot confirm.
  e.ingest(objects(5,LightState::Green)); e.ingest(objects(6,LightState::Green));
  assert(!e.evaluate(ego,.6).stop_required);
  ego.manual_stop=true; assert(!e.evaluate(ego,.6).auto_driving); ego.manual_stop=false;
  ego.manual_stop=true; ego.force_drive=true;
  assert(!e.evaluate(ego,.6).auto_driving);
  ego.manual_stop=false; ego.force_drive=false;
  assert(e.evaluate(ego,2).stop_required); // Live perception timeout.

  e.reset(1); ego.signal=TurnSignal::Left;
  for(int i=1;i<=3;++i) { e.ingest(objects(i)); e.ingest(geometry(i,LineType::Solid)); }
  auto d=e.evaluate(ego,.3); assert(!d.lane_change_allowed && !d.alarm_illegal_lane_change);
  ego.force_lane_change=TurnSignal::Left;
  assert(e.evaluate(ego,.3).alarm_illegal_lane_change);
  ego.force_lane_change=TurnSignal::None;
  for(int i=4;i<=6;++i) { e.ingest(objects(i)); e.ingest(geometry(i)); }
  assert(!e.evaluate(ego,.6).lane_change_allowed); // Missing side != clear.
  auto side=objects(6,LightState::Red,ViewId::Left); e.ingest(side);
  assert(e.evaluate(ego,.6).lane_change_allowed); // Side red doesn't affect front.
  side=objects(7,LightState::Unknown,ViewId::Left); side.vehicle_status=Status::Failed; e.ingest(side);
  assert(!e.evaluate(ego,.7).lane_change_allowed);
  side=objects(8,LightState::Unknown,ViewId::Left); side.obstacles.push_back({10,1,{100,100,300,350},.9f,-1}); e.ingest(side);
  assert(!e.evaluate(ego,.8).lane_change_allowed);
  ego.signal=TurnSignal::None;

  e.reset(1); auto p=objects(1); p.obstacles.push_back({1,7,{280,150,360,350},.9f,-1}); e.ingest(p);
  assert(e.evaluate(ego,.1).brake_now); // No speed/calibration required.
  for(int i=2;i<=3;++i)e.ingest(objects(i));
  assert(e.evaluate(ego,.3).brake_now);
  e.ingest(objects(4)); assert(!e.evaluate(ego,.4).brake_now);
  auto speed=geometry(4); speed.speed={true,80,20,""}; e.ingest(speed);
  assert(!e.evaluate(ego,.4).alarm_overspeed); // Legacy visual speed cannot drive production rules.
  speed=geometry(5); speed.speed.valid=false; e.ingest(speed);
  assert(!e.evaluate(ego,.5).alarm_overspeed);
  ego.use_visual_speed=false; ego.manual_speed_kmh=80;
  assert(e.evaluate(ego,.5).alarm_overspeed);
  e.reset(1); ego.use_visual_speed=true;
  for(int i=0;i<3;++i) {
    auto approaching=objects(i+1);
    approaching.stamp.source_s=approaching.stamp.receive_s=.1+i*.3;
    const float h=2500.f/(25-3*i);
    approaching.obstacles.push_back({10,42,{280,250-h,360,250},.9f,-1});
    e.ingest(approaching);
  }
  assert(e.evaluate(ego,.7).alarm_collision);
  assert(e.evaluate(ego,.7).ttc_s>1.8f && e.evaluate(ego,.7).ttc_s<2.f);
  e.reset(1);ego.signal=TurnSignal::Left;
  for(int i=1;i<=3;++i){e.ingest(objects(i));e.ingest(geometry(i));}
  auto side_traffic=objects(3,LightState::Unknown,ViewId::Left);side_traffic.components=ObjectObservation::Traffic;
  e.ingest(side_traffic);assert(!e.evaluate(ego,.3).lane_change_allowed);
  auto side_vehicle=side_traffic;side_vehicle.components=ObjectObservation::Vehicle;
  assert(e.ingest(side_vehicle)); // Same frame, different model is a new observation.
  assert(!e.evaluate(ego,.3).lane_change_allowed);
  auto side_roadblock=side_traffic;side_roadblock.components=ObjectObservation::Roadblock;
  assert(e.ingest(side_roadblock));
  assert(e.evaluate(ego,.3).lane_change_allowed);
  assert(!e.ingest(side_vehicle));
  auto vehicle_only=objects(10);vehicle_only.components=ObjectObservation::Vehicle;
  vehicle_only.stamp.receive_s=1;vehicle_only.stamp.source_s=1;
  e.ingest(vehicle_only);
  assert(e.evaluate(ego,1).stop_required); // Vehicle refresh cannot revive stale traffic.
  e.reset(2); e.ingest(objects(99,LightState::Green));
  assert(e.evaluate(ego,9.9).stop_required); // Old generation ignored.
  std::cout << "ADAS signal, lane, freshness, dedup, braking and speed: PASS\n";
}
