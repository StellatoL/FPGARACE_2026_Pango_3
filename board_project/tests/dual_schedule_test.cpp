#include "inference/neural_schedule.h"
#include <array>
#include <cassert>
#include <iostream>

int main() {
  using namespace adas;
  NeuralSchedule schedule;
  auto vehicle=schedule.next(Layout::Single,TurnSignal::None,100,1);
  auto traffic=schedule.next(Layout::Single,TurnSignal::None,100,1);
  assert(vehicle&&traffic&&vehicle.kind==NeuralTask::Vehicle&&traffic.kind==NeuralTask::Traffic);
  assert(vehicle.view==ViewId::Front&&traffic.view==ViewId::Front);
  assert(!schedule.next(Layout::Single,TurnSignal::None,200,1));
  assert(!schedule.next(Layout::Single,TurnSignal::None,100.01,2));
  assert(schedule.next(Layout::Single,TurnSignal::None,100.13,2).kind==NeuralTask::Vehicle);
  assert(schedule.next(Layout::Single,TurnSignal::None,100.13,2).kind==NeuralTask::Traffic);
  schedule.reset();
  std::array<std::array<int,4>,2> counts{};int previous=-1;
  for(int i=0;i<10000;++i) {
    auto task=schedule.next(Layout::Mosaic,TurnSignal::None,100+i*.001,uint64_t(i/33+1));
    if(!task)continue;
    const int role=task.kind==NeuralTask::Vehicle?0:1;
    assert(task.kind==NeuralTask::Traffic||task.kind==NeuralTask::Vehicle);
    assert(previous!=role);previous=role;++counts[role][int(task.view)];
  }
  for(auto role:counts) {assert(role[0]>=75&&role[0]<=81);for(int v=1;v<4;++v)assert(role[v]>=19&&role[v]<=21);}
  schedule.reset();counts={};previous=-1;
  for(int i=0;i<10000;++i) {
    auto task=schedule.next(Layout::Mosaic,TurnSignal::Left,100+i*.001,uint64_t(i/33+1));
    if(!task)continue;
    const int role=task.kind==NeuralTask::Vehicle?0:1;
    assert(previous!=role);previous=role;++counts[role][int(task.view)];
  }
  for(auto role:counts)assert(role[0]>=75&&role[3]>=37&&role[1]<=11&&role[2]<=11);
  // Initial equal deadlines prefer the front and then the requested side.
  for(const auto intent:{TurnSignal::Left,TurnSignal::Right}) {
    schedule.reset();
    const auto target=intent==TurnSignal::Left?ViewId::Left:ViewId::Right;
    for(int i=0;i<4;++i) {
      const auto task=schedule.next(Layout::Mosaic,intent,100,1);
      assert(task.view==(i<2?ViewId::Front:target));
      assert(task.kind==(i%2==0?NeuralTask::Vehicle:NeuralTask::Traffic));
    }
  }
  schedule.reset();
  for(int i=0;i<8;++i)assert(schedule.next(Layout::Mosaic,TurnSignal::None,100,1));
  assert(!schedule.next(Layout::Mosaic,TurnSignal::None,200,1));
  schedule.reset();assert(schedule.next(Layout::Single,TurnSignal::None,100,1).kind==NeuralTask::Vehicle);
  std::cout<<"Dual role alternation, independent view clocks, front/target priority and static dedup: PASS\n";
}
