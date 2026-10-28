#include "core/manual_controls.h"
#include "core/adas_engine.h"
#include <cassert>
#include <iostream>
using namespace adas;
int main(){
  EgoState ego;apply_manual_action(ego,ManualAction::ForceDrive);
  assert(ego.force_drive&&!ego.manual_stop);
  ego.automatic=false;apply_manual_action(ego,ManualAction::Drive);assert(!ego.force_drive&&!ego.manual_stop&&ego.automatic);
  apply_manual_action(ego,ManualAction::ForceChangeRight);assert(ego.force_lane_change==TurnSignal::Right);
  apply_manual_action(ego,ManualAction::NavigateLeft);
  assert(ego.force_lane_change==TurnSignal::None&&!ego.force_drive&&ego.signal==TurnSignal::Left&&ego.manual_nav==Direction::Left);
  apply_manual_action(ego,ManualAction::Stop);assert(ego.manual_stop&&!ego.force_drive&&ego.force_lane_change==TurnSignal::None);
  apply_manual_action(ego,ManualAction::NavigateRight);assert(ego.manual_stop); // Route input never cancels a stop.
  apply_manual_action(ego,ManualAction::Drive);assert(!ego.manual_stop);
  apply_manual_action(ego,ManualAction::ClearNavigation);assert(!ego.manual_nav_enabled&&ego.signal==TurnSignal::None);
  for(auto action:{ManualAction::CycleNavigation,ManualAction::NavigateStraight,ManualAction::SignalLeft,ManualAction::SignalRight}){
    ego.force_drive=true;ego.force_lane_change=TurnSignal::Left;apply_manual_action(ego,action);
    assert(!ego.force_drive&&ego.force_lane_change==TurnSignal::None);
  }
  ego={};AdasEngine engine;ObjectObservation scene;
  scene.stamp={1,1,.1,.1,true};scene.width=640;scene.height=360;scene.components=ObjectObservation::All;
  scene.traffic_status=scene.vehicle_status=scene.roadblock_status=Status::Valid;scene.light=LightState::Red;
  engine.ingest(scene,.1);assert(engine.evaluate(ego,.1).stop_required&&!ego.force_drive);
  apply_manual_action(ego,ManualAction::ForceDrive);auto d=engine.evaluate(ego,.1);assert(d.alarm_red_run&&d.auto_driving);
  apply_manual_action(ego,ManualAction::Drive);d=engine.evaluate(ego,.1);assert(!d.alarm_red_run&&!d.auto_driving);
  apply_manual_action(ego,ManualAction::Stop);scene.stamp.sequence=2;scene.light=LightState::Green;engine.ingest(scene,.1);
  assert(!engine.evaluate(ego,.1).auto_driving);
  apply_manual_action(ego,ManualAction::Drive);assert(engine.evaluate(ego,.1).auto_driving);
  std::cout<<"Manual-only force, navigation, static red/green and retained stop: PASS\n";
}
