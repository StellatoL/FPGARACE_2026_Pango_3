#ifndef ADAS_MANUAL_CONTROLS_H
#define ADAS_MANUAL_CONTROLS_H
#include "core/adas_types.h"
namespace adas {
// Only UI/explicit offline operator actions call this function. Perception
// and automatic rules never create a forced action.
enum class ManualAction {Drive,Stop,ForceDrive,NavigateLeft,NavigateStraight,NavigateRight,
  ClearNavigation,CycleNavigation,SignalLeft,SignalRight,ForceChangeLeft,ForceChangeRight,
  ToggleForceLeft,ToggleForceRight,ToggleStop};
inline void apply_manual_action(EgoState &ego,ManualAction action){
  auto ordinary=[&]{ego.force_drive=false;ego.force_lane_change=TurnSignal::None;};
  auto navigate=[&](Direction dir){ordinary();ego.manual_nav_enabled=true;ego.manual_nav=dir;
    ego.signal=dir==Direction::Left?TurnSignal::Left:dir==Direction::Right?TurnSignal::Right:TurnSignal::None;};
  switch(action){
  case ManualAction::Drive:ordinary();ego.manual_stop=false;ego.automatic=true;break;
  case ManualAction::Stop:ordinary();ego.manual_stop=true;break;
  case ManualAction::ForceDrive:ego.manual_stop=false;ego.force_drive=true;ego.force_lane_change=TurnSignal::None;break;
  case ManualAction::NavigateLeft:navigate(Direction::Left);break;
  case ManualAction::NavigateStraight:navigate(Direction::Straight);break;
  case ManualAction::NavigateRight:navigate(Direction::Right);break;
  case ManualAction::ClearNavigation:ordinary();ego.manual_nav_enabled=false;ego.signal=TurnSignal::None;break;
  case ManualAction::CycleNavigation:{
    const int next=ego.manual_nav_enabled?(int(ego.manual_nav)+1)%4:1;
    if(next)navigate(static_cast<Direction>(next));
    else {ordinary();ego.manual_nav_enabled=false;ego.signal=TurnSignal::None;}
    break;}
  case ManualAction::SignalLeft:case ManualAction::SignalRight:{
    ordinary();const auto side=action==ManualAction::SignalLeft?TurnSignal::Left:TurnSignal::Right;
    ego.signal=ego.signal==side?TurnSignal::None:side;break;}
  case ManualAction::ForceChangeLeft:case ManualAction::ForceChangeRight:
  case ManualAction::ToggleForceLeft:case ManualAction::ToggleForceRight:{
    const bool left=action==ManualAction::ForceChangeLeft||action==ManualAction::ToggleForceLeft;
    const auto side=left?TurnSignal::Left:TurnSignal::Right;
    const bool toggle=action==ManualAction::ToggleForceLeft||action==ManualAction::ToggleForceRight;
    ego.force_drive=false;ego.force_lane_change=toggle&&ego.force_lane_change==side?TurnSignal::None:side;
    ego.signal=side;break;}
  case ManualAction::ToggleStop:{const bool stopping=!ego.manual_stop;ordinary();ego.manual_stop=stopping;break;}
  }
}
}
#endif
