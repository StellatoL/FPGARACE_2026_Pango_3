#include "core/adas_engine.h"
#include "core/collision_geometry.h"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace adas;
namespace {
ObjectObservation front(uint64_t sequence,double time) {
  ObjectObservation p;p.stamp={1,sequence,time,time,false};
  p.width=640;p.height=360;p.components=ObjectObservation::All;
  p.traffic_status=p.vehicle_status=p.roadblock_status=Status::Valid;
  p.signal_observation=false;return p;
}
void signal(AdasEngine &engine,uint64_t sequence,double time,LightState color,
            Status status=Status::Valid) {
  SignalObservation p;p.stamp={1,sequence,time,time,false};p.status=status;p.light=color;
  assert(engine.ingest(p,time));
}
void absence_revokes_color_and_force_drive_alarm() {
  for(auto color:{LightState::Red,LightState::Green,LightState::Yellow}) {
    for(auto absent:{LightState::Unknown,LightState::Black}) {
      AdasEngine engine;EgoState ego;ego.force_drive=true;
      for(int i=1;i<=3;++i){engine.ingest(front(i,i*.1));signal(engine,i,i*.1,color);}
      assert(engine.evaluate(ego,.3).light==color);
      engine.ingest(front(4,.4));signal(engine,4,.4,absent);
      auto d=engine.evaluate(ego,.4);
      assert(d.light==LightState::Unknown&&!d.alarm_red_run);
      assert(d.stop_required==(absent==LightState::Black));
      assert(d.auto_driving==(absent==LightState::Unknown)); // Valid empty is ordinary road.
      ego.intersection_mode=true;d=engine.evaluate(ego,.4);
      assert(d.stop_required&&!d.auto_driving); // Explicit junction still needs a signal.
      ego.intersection_mode=false;
      for(int i=5;i<=7;++i) {
        engine.ingest(front(i,i*.1));signal(engine,i,i*.1,color);
        d=engine.evaluate(ego,i*.1);
        assert(d.light==(i==7?color:LightState::Unknown));
        if(i!=7)assert(!d.auto_driving&&!d.alarm_red_run);
      }
    }
  }
}
void failure_and_timeout_hide_all_old_colors() {
  for(auto color:{LightState::Red,LightState::Green,LightState::Yellow}) {
    for(auto outage:{Status::Failed,Status::Missing}) {
      AdasEngine engine;EgoState ego;ego.force_drive=true;
      for(int i=1;i<=3;++i){engine.ingest(front(i,i*.1));signal(engine,i,i*.1,color);}
      engine.ingest(front(4,.4));signal(engine,4,.4,color,outage);
      const auto d=engine.evaluate(ego,.4);
      assert(d.light==LightState::Unknown&&!d.alarm_red_run&&d.stop_required&&!d.auto_driving);
      for(int i=5;i<=7;++i){engine.ingest(front(i,i*.1));signal(engine,i,i*.1,LightState::Green);
        assert(engine.evaluate(ego,i*.1).auto_driving==(i==7));}
    }
    AdasEngine engine;EgoState ego;ego.force_drive=true;
    for(int i=1;i<=3;++i){engine.ingest(front(i,i*.1));signal(engine,i,i*.1,color);}
    engine.ingest(front(4,.81));const auto d=engine.evaluate(ego,.81);
    assert(d.light==LightState::Unknown&&!d.alarm_red_run&&d.stop_required&&!d.auto_driving);
    signal(engine,4,.81,LightState::Green);
    assert(!engine.evaluate(ego,.81).auto_driving);
    engine.ingest(front(5,.91));signal(engine,5,.91,LightState::Green);
    assert(!engine.evaluate(ego,.91).auto_driving);
    engine.ingest(front(6,1.01));signal(engine,6,1.01,LightState::Green);
    assert(engine.evaluate(ego,1.01).auto_driving);
  }
}
void lane_and_view_signal_authority_is_preserved() {
  AdasEngine engine;EgoState ego;ego.force_drive=true;ego.lane=LaneId::Left;
  SignalObservation p;p.status=Status::Valid;p.has_lane_lights=true;
  p.lane_lights={LightState::Red,LightState::Green,LightState::Unknown};
  for(int i=1;i<=3;++i){engine.ingest(front(i,i*.1));p.stamp={1,uint64_t(i),i*.1,i*.1,false};assert(engine.ingest(p));}
  assert(engine.evaluate(ego,.3).alarm_red_run);
  p.stamp={1,4,.4,.4,false};p.lane_lights[0]=LightState::Unknown;
  engine.ingest(front(4,.4));assert(engine.ingest(p));
  auto d=engine.evaluate(ego,.4);
  assert(d.light==LightState::Unknown&&!d.alarm_red_run&&d.stop_required&&!d.auto_driving);
  ego.lane=LaneId::Center;assert(engine.evaluate(ego,.4).auto_driving);
  p.stamp={1,5,.5,.5,false};p.lane_lights={};assert(engine.ingest(p));
  ego.lane=LaneId::Left;assert(engine.evaluate(ego,.5).auto_driving);
  for(auto view:{ViewId::Left,ViewId::Right,ViewId::Rear}) {
    auto side=front(6,.6);side.view=view;side.components|=ObjectObservation::Signal;
    side.signal_status=Status::Valid;side.light=LightState::Red;
    side.has_lane_lights=true;side.lane_lights={LightState::Red,LightState::Red,LightState::Red};
    assert(engine.ingest(side));d=engine.evaluate(ego,.6);
    assert(d.light==LightState::Unknown&&!d.alarm_red_run&&d.auto_driving);
  }
  ego.manual_stop=true;assert(!engine.evaluate(ego,.6).auto_driving);
}
void wide_box_edge_is_not_a_ground_contact() {
  for(int kind:{1,10,11}) {
    for(Box box:{Box{40,100,240,350},Box{160,100,230,350}}) {
      AdasEngine engine;EgoState ego;
      auto p=front(1,.1);p.obstacles={{kind,42,box,.9f,-1}};
      assert(engine.ingest(p,.1));const auto d=engine.evaluate(ego,.1);
      assert(!d.alarm_collision&&!d.brake_now);
    }
  }
}
void substantial_near_footprint_still_counts() {
  for(int kind:{1,10,11}) {
    AdasEngine engine;EgoState ego;
    // Center x=225 lies just outside the near polygon's left edge (~229),
    // but a substantial part of the bottom contact footprint occupies it.
    auto p=front(1,.1);p.obstacles={{kind,42,{100,120,350,350},.9f,-1}};
    assert(engine.ingest(p,.1));const auto d=engine.evaluate(ego,.1);
    assert(d.alarm_collision==(kind!=10)); // Vehicle near risk uses the center gate.
    if(kind==1)assert(d.brake_now);
  }
}
void wide_targets_occupying_the_path_are_not_missed() {
  for(Box box:{Box{0,100,400,350},Box{240,100,640,350}}) {
    assert(CollisionGeometry::ground_in_near(box,640,360));
    assert(CollisionGeometry::ground_in_path(box,640,360));
    for(int kind:{1,10,11}) {
      AdasEngine engine;EgoState ego;
      auto p=front(1,.1);p.obstacles={{kind,42,box,.9f,-1}};
      assert(engine.ingest(p,.1));const auto d=engine.evaluate(ego,.1);
      assert(d.alarm_collision==(kind!=10));
      if(kind!=10)assert(!d.auto_driving);
    }
  }
  // A wide box with only a shallow edge inside the path must still be rejected.
  assert(!CollisionGeometry::ground_in_near({0,100,245,350},640,360));
  assert(!CollisionGeometry::ground_in_near({395,100,640,350},640,360));
}
void near_center_and_independent_far_risks_are_preserved() {
  for(int kind:{1,10,11}) {
    AdasEngine engine;EgoState ego;
    auto p=front(1,.1);p.obstacles={{kind,42,{280,150,360,350},.9f,-1}};
    assert(engine.ingest(p,.1));assert(engine.evaluate(ego,.1).alarm_collision);
  }
  AdasEngine engine;EgoState ego;
  // Contact is above the short near zone, but measured distance still matters.
  auto p=front(1,.1);p.obstacles={{11,42,{280,150,360,240},.9f,2}};
  assert(engine.ingest(p,.1));auto d=engine.evaluate(ego,.1);
  assert(d.alarm_collision&&d.nearest_obstacle_m==2);
  engine.reset(1);
  for(int i=0;i<3;++i) {
    p=front(i+1,.1+i*.3);const float h=2500.f/(25-3*i);
    p.obstacles={{10,42,{280,250-h,360,250},.9f,-1}};
    assert(engine.ingest(p,p.stamp.receive_s));
  }
  d=engine.evaluate(ego,.7);assert(!d.alarm_collision&&d.ttc_s>1.8f&&d.ttc_s<2);
  // The existing 1.2s vehicle cap still warns for a faster approach.
  engine.reset(1);
  for(int i=0;i<3;++i) {
    p=front(i+1,.1+i*.3);const float h=2500.f/(25-5*i);
    p.obstacles={{10,42,{280,250-h,360,250},.9f,-1}};assert(engine.ingest(p));
  }
  d=engine.evaluate(ego,.7);assert(d.alarm_collision&&d.ttc_s>.8f&&d.ttc_s<1.2f);
}
void distant_highway_target_is_not_near_without_ttc_or_distance() {
  AdasEngine engine;EgoState ego;
  auto p=front(1,.1);p.width=1280;p.height=720;
  // Actual 3.367s sample of day-highway: a small distant vehicle on the path.
  p.obstacles={{10,42,{549.6949f,530.9852f,575.8681f,548.5836f},.2567f,-1}};
  assert(engine.ingest(p,.1));const auto d=engine.evaluate(ego,.1);
  assert(!d.alarm_collision&&d.auto_driving&&d.ttc_s<0);
}
void short_geometry_adapts_only_to_usable_fresh_lane() {
  const auto fallback=CollisionGeometry::near_polygon();
  assert(fallback[0].x==.35f&&fallback[0].y==1&&fallback[1].x==.42f&&fallback[1].y==.80f);
  assert(fallback[2].x==.58f&&fallback[3].x==.65f);
  assert(!CollisionGeometry::ground_in_near({280,150,360,250},640,360));
  assert(CollisionGeometry::ground_in_path({280,150,360,250},640,360));
  assert(!CollisionGeometry::ground_in_near({280,150,360,NAN},640,360));
  assert(!CollisionGeometry::ground_in_near({280,150,360,350},0,360));
  LaneObservation road;road.stamp={1,1,.1,.1,false};road.status=Status::Valid;
  road.width=640;road.height=360;auto &l=road.lane;
  l.left_valid=l.right_valid=true;l.left_x=330;l.right_x=550;
  l.left_top_x=400;l.right_top_x=480;l.top_y=180;l.bottom_y=360;
  const Box shifted={420,150,460,350};
  assert(CollisionGeometry::ground_in_near(shifted,640,360,road));
  assert(!CollisionGeometry::ground_in_near(shifted,640,360));
  const auto fitted=CollisionGeometry::near_polygon(road);
  assert(fitted[0].x>l.left_x/640&&fitted[3].x<l.right_x/640);
  assert(fitted[3].x-fitted[0].x<=.300001f);
  AdasEngine engine;EgoState ego;assert(engine.ingest(road));
  auto p=front(1,.1);p.obstacles={{10,42,{280,150,360,350},.9f,-1}};
  assert(engine.ingest(p));assert(engine.evaluate(ego,.1).alarm_collision); // Fixed vehicle near box is independent of lane fit.
  p.stamp={1,2,.7,.7,false};assert(engine.ingest(p));
  assert(engine.evaluate(ego,.7).alarm_collision); // Expired lane also cannot hide center danger.
}
}
int main(int argc,char **argv) {
  const struct {const char *name;void (*run)();} cases[]={
    {"signal absence",absence_revokes_color_and_force_drive_alarm},
    {"signal outage",failure_and_timeout_hide_all_old_colors},
    {"signal authority",lane_and_view_signal_authority_is_preserved},
    {"box edge",wide_box_edge_is_not_a_ground_contact},
    {"substantial footprint",substantial_near_footprint_still_counts},
    {"wide path occupancy",wide_targets_occupying_the_path_are_not_missed},
    {"center and far risk",near_center_and_independent_far_risks_are_preserved},
    {"distant highway target",distant_highway_target_is_not_near_without_ttc_or_distance},
    {"numeric geometry",short_geometry_adapts_only_to_usable_fresh_lane},
  };
  bool selected=false;
  for(const auto &test:cases) {
    if(argc>1&&std::string(argv[1])!=test.name)continue;
    selected=true;std::cout<<test.name<<": "<<std::flush;
    test.run();std::cout<<"PASS\n";
  }
  return selected?0:2;
}
