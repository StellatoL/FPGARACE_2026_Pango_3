#include "core/adas_engine.h"
#include <cassert>
#include <iostream>
using namespace adas;
ObjectObservation frame(int seq,double time){ObjectObservation p;p.stamp={1,uint64_t(seq),time,time,false};
 p.width=640;p.height=360;p.components=ObjectObservation::All;p.traffic_status=p.vehicle_status=p.roadblock_status=Status::Valid;return p;}
GeometryObservation lane(int seq,double time){GeometryObservation g;g.stamp={1,uint64_t(seq),time,time,false};g.components=GeometryObservation::Lane;g.status=Status::Valid;g.width=640;g.height=360;
 g.lane.left_valid=g.lane.right_valid=true;g.lane.left_type=g.lane.right_type=LineType::Dashed;
 g.lane.left_x=210;g.lane.right_x=430;g.lane.left_top_x=280;g.lane.right_top_x=360;g.lane.top_y=180;g.lane.bottom_y=360;return g;}
Obstacle prediction(int kind,Box box,double observed,float confidence=.9f){Obstacle o{kind,42,box,confidence,-1};o.predicted=true;o.last_observed_s=observed;o.track_hits=2;return o;}
int main(){
 EgoState ego;ego.input_layout=Layout::Single;AdasEngine engine;
 for(int i=1;i<=2;++i){auto p=frame(i,i*.1);p.obstacles={{1,42,{280,190,350,288},.5f,-1}};engine.ingest(p,i*.1);}
 assert(!engine.evaluate(ego,.2).brake_now);
 auto p=frame(3,.3);p.obstacles={prediction(1,{280,190,350,288},.2)};engine.ingest(p,.3);
 assert(!engine.evaluate(ego,.3).brake_now); // Predictions cannot become a third measured vote.
 engine.reset(1);
 p=frame(1,.1);p.obstacles={{1,42,{280,190,350,288},.9f,-1}};engine.ingest(p,.1);assert(engine.evaluate(ego,.1).brake_now);
 for(int i=2;i<=4;++i){p=frame(i,.1+i*.05);p.obstacles={prediction(1,{10,10,40,90},.1,.1f)};engine.ingest(p,p.stamp.receive_s);}
 assert(engine.evaluate(ego,.3).brake_now); // Predicted movement/decay is not observed clearance.
 for(int i=5;i<=7;++i){p=frame(i,.31+(i-5)*.01);engine.ingest(p,p.stamp.receive_s);}
 assert(!engine.evaluate(ego,.33).brake_now);
 engine.reset(1);ego.signal=TurnSignal::Left;
 for(int i=1;i<=3;++i){engine.ingest(frame(i,i*.1));engine.ingest(lane(i,i*.1));}
 p=frame(4,.31);p.obstacles={prediction(11,{50,200,190,300},.3,.1f)};engine.ingest(p,.31);
 assert(!engine.evaluate(ego,.4).lane_change_allowed); // A low confidence coast still vetoes.
 assert(engine.evaluate(ego,.7).lane_change_allowed); // Cached prediction must expire at consumption.
 engine.reset(1);ego.signal=TurnSignal::None;
 for(int i=1;i<=2;++i){p=frame(i,i*.1);p.obstacles={{10,42,{280,190,360,250},.9f,10}};engine.ingest(p,i*.1);}
 p=frame(3,.3);auto fake=prediction(10,{280,180,360,260},.2);fake.distance_m=1;p.obstacles={fake};engine.ingest(p,.3);
 const auto result=engine.evaluate(ego,.3);assert(result.ttc_s<0&&result.nearest_obstacle_m<0&&!result.alarm_collision);
 std::cout<<"Predictions cannot vote, clear danger, measure TTC/distance or outlive consumer TTL: PASS\n";
}
