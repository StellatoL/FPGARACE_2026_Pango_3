#ifndef ADAS_NEURAL_SCHEDULE_H
#define ADAS_NEURAL_SCHEDULE_H
#include "core/adas_types.h"
#include <array>
#include <cmath>
namespace adas {
struct NeuralTask {
  enum Kind {Scene,Lane,Navigation,Signal};
  Kind kind=Scene;ViewId view=ViewId::Front;int slot=-1;
  explicit operator bool()const{return slot>=0;}
};
class NeuralSchedule {
public:
  NeuralTask next(Layout layout,TurnSignal intent,double now,uint64_t sequence) {
    NeuralTask best;double earliest=1e30;
    for(int i=0;i<7;++i) {
      if(layout==Layout::Single&&i>0&&i<4)continue;
      if(sequence<=sequence_[i])continue;
      const double due=started_[i]?time_[i]+period(i,layout,intent):0;
      if(due<=now&&due<earliest){earliest=due;best={i<4?NeuralTask::Scene:i==4?NeuralTask::Lane:i==5?NeuralTask::Navigation:NeuralTask::Signal,i<4?static_cast<ViewId>(i):ViewId::Front,i};}
    }
    if(best){const auto i=best.slot;started_[i]=true;time_[i]=now;sequence_[i]=sequence;}
    return best;
  }
  void reset(){time_={};sequence_={};started_={};}
private:
  static double period(int i,Layout layout,TurnSignal intent) {
    if(i==0)return layout==Layout::Single?1./12:1./8;
    if(i==4)return layout==Layout::Single?1./12:1./10;
    if(i>=5)return .25; // Independent authoritative ROIs must fit the 500ms freshness window.
    if(intent==TurnSignal::None)return .5;
    const int target=intent==TurnSignal::Left?3:1;
    return i==target?.25:1.;
  }
  std::array<double,7> time_{};std::array<uint64_t,7> sequence_{};std::array<bool,7> started_{};
};
}
#endif
