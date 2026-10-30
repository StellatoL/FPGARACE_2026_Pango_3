#include "core/adas_engine.h"
#include <cassert>
#include <iostream>
using namespace adas;
ObjectObservation objects(uint64_t seq,double t){ObjectObservation p;p.stamp={1,seq,t,t,false};p.width=640;p.height=360;p.components=ObjectObservation::All;p.traffic_status=p.vehicle_status=p.roadblock_status=Status::Valid;p.signal_observation=false;return p;}
LaneObservation road(){LaneObservation p;p.stamp={1,1,.1,.1,false};p.status=Status::Valid;p.width=640;p.height=360;auto &l=p.lane;l.left_valid=l.right_valid=true;l.left_type=l.right_type=LineType::Dashed;l.left_x=210;l.right_x=430;l.left_top_x=280;l.right_top_x=360;l.top_y=180;l.bottom_y=360;return p;}
int main(){AdasEngine e;EgoState ego;ego.input_layout=Layout::Single;
 auto p=objects(1,.1);p.obstacles={{10,42,{280,150,360,250},.9f,-1}};e.ingest(p);e.ingest(road());auto d=e.evaluate(ego,.1);
 assert(d.slowdown_warning&&d.stop_required&&!d.auto_driving&&!d.lane_change_allowed);assert(d.status_text.find(u8"减速")!=std::string::npos);
 ego.intersection_mode=true;d=e.evaluate(ego,.1);assert(!d.slowdown_warning&&d.stop_required);ego.intersection_mode=false;
 ego.manual_stop=true;assert(!e.evaluate(ego,.1).slowdown_warning);ego.manual_stop=false;
 ego.paused=true;assert(!e.evaluate(ego,.1).slowdown_warning);ego.paused=false;
 assert(!e.evaluate(ego,1).slowdown_warning);
 SignalObservation light;light.stamp={1,1,.1,.1,true};light.status=Status::Valid;light.light=LightState::Red;e.ingest(light);
 assert(!e.evaluate(ego,.1).slowdown_warning&&e.evaluate(ego,.1).stop_required);
 e.reset(1);p=objects(1,.1);p.obstacles={{1,42,{280,160,360,300},.9f,-1}};e.ingest(p);d=e.evaluate(ego,.1);assert(d.brake_now&&d.stop_required&&!d.slowdown_warning);
 e.reset(1);p=objects(1,.1);p.obstacles={{10,42,{280,150,360,350},.9f,-1}};e.ingest(p);d=e.evaluate(ego,.1);assert(d.alarm_collision&&d.slowdown_warning&&d.stop_required&&!d.auto_driving&&!d.lane_change_allowed);
 std::cout<<"PASS slowdown presentation never overrides signal, stale input, manual stop or pedestrian braking\n";
}
