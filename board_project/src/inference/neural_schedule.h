#ifndef ADAS_NEURAL_SCHEDULE_H
#define ADAS_NEURAL_SCHEDULE_H
#include "core/adas_types.h"
#include <array>
#include <cmath>
namespace adas {
struct NeuralTask {
  enum Kind {Traffic,Vehicle};
  Kind kind=Traffic;ViewId view=ViewId::Front;int slot=-1;
  explicit operator bool()const{return slot>=0;}
};
class NeuralSchedule {
public:
  NeuralTask next(Layout layout,TurnSignal intent,double now,uint64_t sequence) {
    if(!std::isfinite(now)||now<0||!sequence)return {};
    NeuralTask best;double earliest=1e30;
    const int role=dual_role_;
    for(int view=0;view<(layout==Layout::Single?1:4);++view) {
      if(sequence<=dual_sequence_[role][view])continue;
      const double due=dual_started_[role][view]?dual_time_[role][view]+dual_period(view,intent):0;
      const auto priority=[&](int candidate){
        if(candidate==0)return 0;
        if(intent!=TurnSignal::None&&candidate==(intent==TurnSignal::Left?3:1))return 1;
        return candidate+2;
      };
      if(due<=now&&(due<earliest||(due==earliest&&best&&priority(view)<priority(int(best.view))))) {
        earliest=due;best={role==0?NeuralTask::Vehicle:NeuralTask::Traffic,static_cast<ViewId>(view),role*4+view};
      }
    }
    if(best) {
      const int view=int(best.view);
      dual_started_[role][view]=true;dual_time_[role][view]=now;dual_sequence_[role][view]=sequence;
      dual_role_=1-role;
    }
    return best;
  }
  static double dual_period(int view,TurnSignal intent) {
    if(view==0)return 1./8;
    if(intent==TurnSignal::None)return .5;
    return view==(intent==TurnSignal::Left?3:1)?.25:1.;
  }
  void reset(){dual_time_={};dual_sequence_={};dual_started_={};dual_role_=0;}
private:
  std::array<std::array<double,4>,2> dual_time_{};
  std::array<std::array<uint64_t,4>,2> dual_sequence_{};
  std::array<std::array<bool,4>,2> dual_started_{};
  int dual_role_=0;
};
}
#endif
