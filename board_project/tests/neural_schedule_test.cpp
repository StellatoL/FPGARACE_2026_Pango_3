#include "inference/neural_schedule.h"
#include <cassert>
#include <array>
#include <iostream>
int main(){
  adas::NeuralSchedule s;std::array<int,7> count{};
  for(int i=0;i<10000;++i){auto t=s.next(adas::Layout::Mosaic,adas::TurnSignal::None,i*.001,uint64_t(i/33+1));if(t)++count[t.slot];}
  assert(count[0]>=75&&count[0]<=81);for(int i=1;i<=3;++i)assert(count[i]>=19&&count[i]<=21);
  assert(count[4]>=94&&count[4]<=101);assert(count[5]>=38&&count[5]<=41&&count[6]>=38&&count[6]<=41);
  s.reset();count={};for(int i=0;i<10000;++i){auto t=s.next(adas::Layout::Mosaic,adas::TurnSignal::Right,i*.001,uint64_t(i/33+1));if(t)++count[t.slot];}
  assert(count[1]>=37&&count[2]<=11&&count[3]<=11&&count[0]>=75);
  s.reset();count={};for(int i=0;i<10000;++i){auto t=s.next(adas::Layout::Single,adas::TurnSignal::None,i*.001,uint64_t(i/33+1));if(t)++count[t.slot];}
  assert(count[0]>=110&&count[4]>=110&&count[1]+count[2]+count[3]==0);
  s.reset();for(int i=0;i<7;++i)assert(s.next(adas::Layout::Mosaic,adas::TurnSignal::None,100,1));
  assert(!s.next(adas::Layout::Mosaic,adas::TurnSignal::None,200,1));
  std::cout<<"neural schedule passed\n";
}
