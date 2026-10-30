#include "core/adas_engine.h"
#include <cassert>
#include <iostream>

using namespace adas;

namespace {
ObjectObservation primary(uint64_t sequence, double time,
                          unsigned components = ObjectObservation::Combined,
                          ViewId view = ViewId::Front, bool still = false) {
  ObjectObservation p;
  p.stamp = {1, sequence, time, time, still};
  p.components = components; p.view = view; p.width = 640; p.height = 360;
  p.traffic_status = p.vehicle_status = Status::Valid;
  p.signal_observation = false;
  return p;
}
ObjectObservation crossing(uint64_t sequence, double time, bool danger,
                           ViewId view = ViewId::Front, bool still = false) {
  auto p = primary(sequence, time, ObjectObservation::Crosswalk, view, still);
  p.crosswalk_status = Status::Valid;
  p.has_zebra = true; p.person_on_zebra = danger;
  return p;
}
LaneObservation lane(uint64_t sequence, double time, bool still = false,
                     LineType type = LineType::Dashed) {
  LaneObservation l;
  l.stamp = {1, sequence, time, time, still}; l.status = Status::Valid;
  l.width = 640; l.height = 360;
  l.lane.left_valid = l.lane.right_valid = true;
  l.lane.left_type = l.lane.right_type = type;
  l.lane.left_x = 210; l.lane.right_x = 430;
  l.lane.left_top_x = 280; l.lane.right_top_x = 360;
  l.lane.top_y = 180; l.lane.bottom_y = 360;
  return l;
}
void dual(AdasEngine &engine) {
  engine.required_obstacle_components = ObjectObservation::Combined;
  engine.separate_crosswalk_source = true;
}
void confirm_crossing(AdasEngine &engine, uint64_t first, double time) {
  for (uint64_t i = 0; i < 3; ++i)
    assert(engine.ingest(crossing(first + i, time + .05 * i, true)));
}
void capability_is_explicit_and_unknown_masks_fail_closed() {
  EgoState ego; ego.input_layout = Layout::Single;
  AdasEngine engine;
  assert(engine.required_obstacle_components == ObjectObservation::All);
  assert(!engine.separate_crosswalk_source);
  auto p = primary(1, .1);
  assert(p.roadblock_status == Status::Missing);
  assert(engine.ingest(p));
  assert(engine.evaluate(ego, .1).stop_required);
  dual(engine);
  assert(!engine.evaluate(ego, .1).stop_required);
  assert(p.roadblock_status == Status::Missing);
  for (unsigned mask : {0u, 1u, 2u, 4u, 8u, 16u, 31u, 0xffffffffu}) {
    engine.required_obstacle_components = mask;
    assert(engine.evaluate(ego, .1).stop_required);
    ego.force_drive = true;
    assert(!engine.evaluate(ego, .1).auto_driving);
    ego.force_drive = false;
  }
  engine.required_obstacle_components = ObjectObservation::All;
  p.stamp.sequence = 2; p.components = ObjectObservation::Roadblock;
  p.roadblock_status = Status::Valid;
  assert(engine.ingest(p));
  assert(!engine.evaluate(ego, .1).stop_required);
}
void each_primary_component_must_be_valid_and_fresh() {
  const unsigned kinds[] = {ObjectObservation::Traffic, ObjectObservation::Vehicle};
  for (unsigned kind : kinds) {
    AdasEngine engine; dual(engine); EgoState ego;
    const unsigned other = ObjectObservation::Combined ^ kind;
    assert(engine.ingest(primary(1, .1, other)));
    assert(engine.evaluate(ego, .1).stop_required);
    auto p = primary(1, .1, kind);
    if (kind == ObjectObservation::Traffic) p.traffic_status = Status::Failed;
    else p.vehicle_status = Status::Failed;
    assert(engine.ingest(p)); assert(engine.evaluate(ego, .1).stop_required);
    p = primary(2, .2, kind); assert(engine.ingest(p));
    assert(!engine.evaluate(ego, .2).stop_required);
    assert(engine.ingest(primary(3, .7, other)));
    assert(!engine.evaluate(ego, .7).stop_required);
    assert(engine.evaluate(ego, .701).stop_required);
    // Neither the separate traffic model nor its signals freshen people/vehicles.
    auto x = crossing(50, .71, false);
    x.components |= ObjectObservation::Signal;
    x.signal_status = Status::Valid; x.light = LightState::Unknown;
    assert(engine.ingest(x));
    assert(engine.evaluate(ego, .71).stop_required);
    p = primary(4, .72, kind); assert(engine.ingest(p));
    assert(!engine.evaluate(ego, .72).stop_required);
  }
}
void mosaic_requires_both_requested_side_components() {
  for (TurnSignal turn : {TurnSignal::Left, TurnSignal::Right}) {
    const ViewId side = turn == TurnSignal::Left ? ViewId::Left : ViewId::Right;
    for (unsigned kind : {ObjectObservation::Traffic, ObjectObservation::Vehicle}) {
      AdasEngine engine; dual(engine); EgoState ego;
      ego.signal = turn; ego.lane_change_request_s = .25;
      for (int i = 1; i <= 3; ++i) {
        assert(engine.ingest(primary(i, i * .1)));
        assert(engine.ingest(lane(i, i * .1)));
      }
      assert(engine.ingest(primary(1, .25, ObjectObservation::Combined, side)));
      assert(!engine.evaluate(ego, .3).lane_change_allowed);
      assert(engine.ingest(primary(2, .3, ObjectObservation::Combined ^ kind, side)));
      assert(!engine.evaluate(ego, .3).lane_change_allowed);
      auto x = crossing(100, .31, false, side);
      assert(engine.ingest(x));
      assert(!engine.evaluate(ego, .31).lane_change_allowed);
      auto p = primary(2, .32, kind, side);
      assert(engine.ingest(p));
      assert(engine.evaluate(ego, .32).lane_change_allowed);
      p = primary(3, .33, kind, side);
      if (kind == ObjectObservation::Traffic) p.traffic_status = Status::Failed;
      else p.vehicle_status = Status::Failed;
      assert(engine.ingest(p)); assert(!engine.evaluate(ego, .33).lane_change_allowed);
      p = primary(4, .34, kind, side);
      p.obstacles = {{kind == ObjectObservation::Traffic ? 1 : 10, 4,
                      {30, 170, 150, 300}, .9f, -1}};
      assert(engine.ingest(p)); assert(!engine.evaluate(ego, .34).lane_change_allowed);
      p = primary(5, .35, kind, side);
      assert(engine.ingest(p)); assert(engine.evaluate(ego, .35).lane_change_allowed);
      // Refresh the opposite side component and front only. This source still expires.
      assert(engine.ingest(primary(6, 1.86)));
      for (int i = 4; i <= 6; ++i) assert(engine.ingest(lane(i, 1.84 + (i - 4) * .01)));
      assert(engine.ingest(primary(6, 1.86, ObjectObservation::Combined ^ kind, side)));
      assert(!engine.evaluate(ego, 1.86).lane_change_allowed);
    }
  }
}
void static_source_is_ingested_once_and_manual_rules_still_apply() {
  AdasEngine engine; dual(engine); EgoState ego; ego.input_layout = Layout::Single;
  auto p = primary(1, .1, ObjectObservation::Combined, ViewId::Front, true);
  auto l = lane(1, .1, true);
  assert(engine.ingest(p)); assert(engine.ingest(l));
  ego.signal = TurnSignal::Left;
  for (int i = 0; i < 20; ++i) {
    assert(!engine.ingest(p)); assert(!engine.ingest(l));
    assert(engine.evaluate(ego, 100 + i).lane_change_allowed);
  }
  ego.manual_stop = true; assert(!engine.evaluate(ego, 120).lane_change_allowed);
  ego.manual_stop = false; ego.paused = true;
  assert(!engine.evaluate(ego, 120).lane_change_allowed); ego.paused = false;
  ego.input_layout = Layout::Mosaic;
  assert(!engine.evaluate(ego, 120).lane_change_allowed);
  p.view = ViewId::Left; assert(engine.ingest(p));
  assert(engine.evaluate(ego, 120).lane_change_allowed);
  l = lane(2, .1, true, LineType::Solid); assert(engine.ingest(l));
  ego.force_lane_change = TurnSignal::Left;
  const auto d = engine.evaluate(ego, 120);
  assert(d.alarm_illegal_lane_change && !d.lane_change_allowed);
}
void crosswalk_brakes_immediately_and_primary_cannot_clear() {
  AdasEngine engine; dual(engine); EgoState ego;
  assert(engine.ingest(crossing(1, .1, true)));
  assert(engine.evaluate(ego, .1).brake_now); // The first measured crossing is urgent.
  for (int i = 1; i <= 6; ++i) {
    auto p = primary(i, .1 + i * .01);
    p.person_on_zebra = true; // This field has no authority in the primary source.
    assert(engine.ingest(p));
    assert(engine.evaluate(ego, p.stamp.receive_s).brake_now);
  }
  assert(engine.ingest(crossing(2, .2, true)));
  for (int i = 0; i < 20; ++i) {
    assert(!engine.ingest(crossing(2, .2, true)));
    assert(engine.evaluate(ego, .2).brake_now);
  }
  assert(engine.ingest(crossing(3, .3, true)));
  assert(engine.evaluate(ego, .3).brake_now);
  for (int i = 7; i <= 10; ++i) assert(engine.ingest(primary(i, .3 + i * .01)));
  assert(engine.evaluate(ego, .4).brake_now);
  assert(engine.ingest(crossing(4, .41, false)));
  assert(engine.ingest(crossing(5, .42, false)));
  assert(engine.evaluate(ego, .42).brake_now);
  assert(engine.ingest(crossing(6, .43, false)));
  assert(!engine.evaluate(ego, .43).brake_now);
}
void primary_person_brakes_and_has_an_independent_clear_latch() {
  AdasEngine engine; dual(engine); EgoState ego;
  auto p = primary(1, .1);
  p.obstacles = {{1, 1, {280, 180, 360, 290}, .9f, -1}};
  assert(engine.ingest(p)); assert(engine.evaluate(ego, .1).brake_now);
  for (int i = 1; i <= 5; ++i) assert(engine.ingest(crossing(i, .1 + i * .01, false)));
  assert(engine.evaluate(ego, .15).brake_now);
  assert(engine.ingest(primary(2, .2))); assert(engine.ingest(primary(3, .3)));
  assert(engine.evaluate(ego, .3).brake_now);
  assert(engine.ingest(primary(4, .4))); assert(!engine.evaluate(ego, .4).brake_now);
  // A lower-confidence person keeps the original three distinct measured votes.
  engine.reset(1);
  for (int i = 1; i <= 3; ++i) {
    p = primary(i, i * .1);
    p.obstacles = {{1, 1, {280, 180, 360, 290}, .5f, -1}};
    assert(engine.ingest(p)); assert(engine.evaluate(ego, i * .1).brake_now == (i == 3));
  }
}
void failed_stale_and_duplicate_crossing_cannot_release_danger() {
  AdasEngine engine; dual(engine); EgoState ego;
  confirm_crossing(engine, 1, .1); assert(engine.evaluate(ego, .2).brake_now);
  assert(engine.ingest(crossing(4, .21, false)));
  auto p = crossing(5, .22, false); p.crosswalk_status = Status::Failed;
  assert(engine.ingest(p));
  assert(engine.ingest(crossing(6, .23, false)));
  assert(engine.ingest(crossing(7, .24, false)));
  assert(engine.evaluate(ego, .24).brake_now);
  p = crossing(8, .25, false); assert(engine.ingest(p, 1.));
  assert(engine.evaluate(ego, 1.).brake_now);
  // Stale clears reset the sequence: three new fresh clears are required.
  assert(engine.ingest(crossing(9, 1.01, false)));
  for (int i = 0; i < 20; ++i) {
    assert(!engine.ingest(crossing(9, 1.01, false)));
    assert(engine.evaluate(ego, 1.01).brake_now);
  }
  auto old = crossing(8, 1.02, false); assert(!engine.ingest(old));
  old = crossing(100, 1.02, false); old.stamp.generation = 0;
  assert(!engine.ingest(old));
  assert(engine.ingest(crossing(10, 1.02, false)));
  assert(engine.evaluate(ego, 1.02).brake_now);
  assert(engine.ingest(crossing(11, 1.03, false)));
  assert(!engine.evaluate(ego, 1.03).brake_now);
}
void crosswalk_danger_remains_immediate_after_an_outage() {
  for (Status interrupt : {Status::Missing, Status::Failed, Status::Valid}) {
    AdasEngine engine; dual(engine); EgoState ego;
    assert(engine.ingest(crossing(1, .1, true)));
    assert(engine.ingest(crossing(2, .2, true)));
    auto p = crossing(3, .3, true); p.crosswalk_status = interrupt;
    assert(engine.ingest(p, interrupt == Status::Valid ? 1. : .3));
    const double time = interrupt == Status::Valid ? 1.1 : .4;
    for (int i = 0; i < 3; ++i) {
      assert(engine.ingest(crossing(4 + i, time + i * .1, true)));
      assert(engine.evaluate(ego, time + i * .1).brake_now);
    }
  }
}
void nonfront_crosswalk_or_signal_cannot_brake_front_or_replace_people() {
  AdasEngine engine; dual(engine); EgoState ego; ego.input_layout = Layout::Single;
  assert(engine.ingest(primary(1, .1, ObjectObservation::Combined, ViewId::Front, true)));
  for (ViewId view : {ViewId::Left, ViewId::Right, ViewId::Rear}) {
    auto p = crossing(1, .1, true, view, true);
    p.components |= ObjectObservation::Signal; p.signal_status = Status::Valid;
    p.light = LightState::Red;
    p.obstacles = {{1, 7, {280, 170, 360, 350}, .99f, 1}};
    assert(engine.ingest(p));
    const auto d = engine.evaluate(ego, 100);
    assert(!d.brake_now && !d.alarm_collision && !d.stop_required);
    assert(d.light == LightState::Unknown);
  }
  // Front traffic-model people are auxiliary: only crossing risk is authoritative.
  auto x = crossing(1, .1, false, ViewId::Front, true);
  x.obstacles = {{1, 8, {280, 170, 360, 350}, .99f, 1}};
  x.width = x.height = 0; // Independent metadata cannot erase primary image geometry.
  assert(engine.ingest(x));
  assert(!engine.evaluate(ego, 100).brake_now);
  assert(!engine.evaluate(ego, 100).alarm_collision);
  assert(!engine.evaluate(ego, 100).stop_required);
  auto p = primary(2, .1, ObjectObservation::Traffic, ViewId::Front, true);
  p.obstacles = {{1, 9, {280, 170, 360, 350}, .99f, 1}};
  assert(engine.ingest(p)); assert(engine.evaluate(ego, 100).brake_now);
  x.stamp.sequence = 2; assert(engine.ingest(x));
  assert(engine.evaluate(ego, 100).brake_now);
}
void reset_preserves_profile_and_clears_all_crosswalk_history() {
  AdasEngine engine; dual(engine); EgoState ego;
  engine.front_timeout_s = .8; engine.side_timeout_s = 2;
  engine.external_speed_timeout_s = 3; engine.ttc_threshold_s = 1.8f;
  confirm_crossing(engine, 1, .1); assert(engine.evaluate(ego, .2).brake_now);
  engine.reset(2);
  assert(engine.required_obstacle_components == ObjectObservation::Combined);
  assert(engine.separate_crosswalk_source && engine.front_timeout_s == .8);
  assert(engine.side_timeout_s == 2 && engine.external_speed_timeout_s == 3);
  assert(engine.ttc_threshold_s == 1.8f && !engine.evaluate(ego, .2).brake_now);
  assert(engine.evaluate(ego, .2).stop_required);
  auto p = crossing(1, .3, true); assert(!engine.ingest(p));
  p.stamp.generation = 2; assert(engine.ingest(p));
  assert(engine.evaluate(ego, .3).brake_now);
  auto source = primary(1, .3); source.stamp.generation = 2;
  assert(engine.ingest(source)); assert(engine.evaluate(ego, .3).stop_required);
  engine.reset(1); assert(engine.generation() == 2);
  // Automatic source transitions preserve the explicitly selected profile too.
  source.stamp.generation = 3; assert(engine.ingest(source));
  assert(engine.required_obstacle_components == ObjectObservation::Combined);
  assert(engine.separate_crosswalk_source && !engine.evaluate(ego, .3).stop_required);
}
} // namespace

int main(int argc, char **argv) {
  const struct { const char *name; void (*run)(); } cases[] = {
    {"capability policy", capability_is_explicit_and_unknown_masks_fail_closed},
    {"independent primary freshness", each_primary_component_must_be_valid_and_fresh},
    {"post-request side capability", mosaic_requires_both_requested_side_components},
    {"static and manual rules", static_source_is_ingested_once_and_manual_rules_still_apply},
    {"immediate crossing and independent clear", crosswalk_brakes_immediately_and_primary_cannot_clear},
    {"independent primary person brake", primary_person_brakes_and_has_an_independent_clear_latch},
    {"crosswalk invalid, stale, duplicate rejection", failed_stale_and_duplicate_crossing_cannot_release_danger},
    {"immediate crossing after outage", crosswalk_danger_remains_immediate_after_an_outage},
    {"view and source isolation", nonfront_crosswalk_or_signal_cannot_brake_front_or_replace_people},
    {"profile-preserving reset", reset_preserves_profile_and_clears_all_crosswalk_history},
  };
  bool selected = false;
  for (const auto &test : cases) {
    if (argc > 1 && std::string(argv[1]) != test.name) continue;
    selected = true;
    std::cout << test.name << ": " << std::flush;
    test.run(); std::cout << "PASS\n";
  }
  return selected ? 0 : 2;
}
