#include "core/view_tracker.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace adas;

#define CHECK(expression) do { if (!(expression)) throw std::runtime_error(\
  std::string(__func__) + ":" + std::to_string(__LINE__) + " " #expression); } while (false)

namespace {
Obstacle person(float x = 100.f, int class_id = 1) {
  return {class_id, -1, {x, 100, x + 40, 220}, .9f, 8.f};
}
ObjectObservation sample(uint64_t seq, double time, std::vector<Obstacle> objects = {},
                         ViewId view = ViewId::Front,
                         unsigned components = ObjectObservation::Traffic) {
  ObjectObservation observation;
  observation.stamp = {1, seq, time, time, false};
  observation.view = view;
  observation.components = components;
  observation.traffic_status = observation.vehicle_status = observation.roadblock_status = Status::Valid;
  observation.width = 2000; observation.height = 1000;
  observation.obstacles = std::move(objects);
  return observation;
}
int64_t confirmed(ViewTracker &tracker, ViewId view = ViewId::Front, int class_id = 1) {
  const unsigned component = class_id == 1 ? ObjectObservation::Traffic :
                             class_id == 10 ? ObjectObservation::Vehicle : ObjectObservation::Roadblock;
  auto first = sample(1, .1, {person(100, class_id)}, view, component);
  auto second = sample(2, .2, {person(110, class_id)}, view, component);
  CHECK(tracker.update(first)); CHECK(tracker.update(second));
  CHECK(first.obstacles.size() == 1 && second.obstacles.size() == 1);
  CHECK(first.obstacles[0].track_id == second.obstacles[0].track_id);
  return second.obstacles[0].track_id;
}

void two_hits_predict_and_recover_motion() {
  ViewTracker tracker;
  const auto id = confirmed(tracker);
  auto first_miss = sample(3, .3); CHECK(tracker.update(first_miss));
  CHECK(first_miss.obstacles.size() == 1);
  CHECK(first_miss.obstacles[0].track_id == id);
  CHECK(std::abs(first_miss.obstacles[0].box.left - 120.f) < .01f);
  CHECK(first_miss.obstacles[0].distance_m == -1);
  CHECK(first_miss.obstacles[0].confidence < .9f);
  auto second_miss = sample(4, .4); CHECK(tracker.update(second_miss));
  CHECK(second_miss.obstacles.size() == 1);
  CHECK(std::abs(second_miss.obstacles[0].box.left - 130.f) < .01f);
  CHECK(second_miss.obstacles[0].confidence < first_miss.obstacles[0].confidence);
  auto recovered = sample(5, .5, {person(140)}); CHECK(tracker.update(recovered));
  CHECK(recovered.obstacles.size() == 1);
  CHECK(recovered.obstacles[0].track_id == id);
  CHECK(recovered.obstacles[0].distance_m == 8.f);
}
void one_or_nonconsecutive_hits_do_not_predict() {
  ViewTracker tracker;
  auto first = sample(1, .1, {person()}); CHECK(tracker.update(first));
  auto miss = sample(2, .2); CHECK(tracker.update(miss)); CHECK(miss.obstacles.empty());
  auto again = sample(3, .3, {person()}); CHECK(tracker.update(again));
  miss = sample(4, .4); CHECK(tracker.update(miss)); CHECK(miss.obstacles.empty());
}
void front_and_side_expiry_are_bounded() {
  ViewTracker front;
  const auto front_id = confirmed(front);
  auto expired = sample(3, .56); CHECK(front.update(expired)); CHECK(expired.obstacles.empty());
  auto again = sample(4, .57, {person(110)}); CHECK(front.update(again));
  CHECK(again.obstacles[0].track_id != front_id);
  for (const auto view : {ViewId::Left, ViewId::Right, ViewId::Rear}) {
    ViewTracker side; confirmed(side, view);
    auto alive = sample(3, .7, {}, view); CHECK(side.update(alive));
    CHECK(alive.obstacles.size() == 1);
    auto dead = sample(4, .96, {}, view); CHECK(side.update(dead)); CHECK(dead.obstacles.empty());
  }
}
void missed_frame_limit_applies_at_high_frame_rate() {
  ViewTracker tracker(2);
  const auto id = confirmed(tracker);
  for (uint64_t sequence = 3; sequence <= 4; ++sequence) {
    auto miss = sample(sequence, .2 + .01 * (sequence - 2));
    CHECK(tracker.update(miss)); CHECK(miss.obstacles.size() == 1);
  }
  auto expired = sample(5, .23); CHECK(tracker.update(expired)); CHECK(expired.obstacles.empty());
  auto recovered = sample(6, .24, {person(110)}); CHECK(tracker.update(recovered));
  CHECK(recovered.obstacles[0].track_id != id);
}
void view_class_and_partial_components_are_isolated() {
  ViewTracker tracker;
  auto first = sample(1, .1, {person(100, 1), person(100, 10), person(100, 11)},
                      ViewId::Front, ObjectObservation::All);
  auto second = sample(2, .2, first.obstacles, ViewId::Front, ObjectObservation::All);
  CHECK(tracker.update(first)); CHECK(tracker.update(second));
  CHECK(first.obstacles[0].track_id != first.obstacles[1].track_id);
  CHECK(first.obstacles[1].track_id != first.obstacles[2].track_id);
  auto other_view = sample(2, .2, {person()}, ViewId::Left); CHECK(tracker.update(other_view));
  CHECK(other_view.obstacles[0].track_id != first.obstacles[0].track_id);
  auto only_vehicle = sample(3, .3, {}, ViewId::Front, ObjectObservation::Vehicle);
  CHECK(tracker.update(only_vehicle)); CHECK(only_vehicle.obstacles.size() == 1);
  CHECK(only_vehicle.obstacles[0].class_id == 10);
  auto traffic_same_sequence = sample(3, .3); CHECK(tracker.update(traffic_same_sequence));
  CHECK(traffic_same_sequence.obstacles.size() == 1);
  CHECK(traffic_same_sequence.obstacles[0].class_id == 1);
}
void failed_or_missing_components_never_gain_predictions() {
  for (auto status : {Status::Failed, Status::Missing}) {
    ViewTracker tracker; const auto id = confirmed(tracker);
    auto failed = sample(3, .3); failed.traffic_status = status;
    CHECK(tracker.update(failed)); CHECK(failed.traffic_status == status);
    CHECK(failed.obstacles.empty());
    auto old_failed = sample(4, .7); old_failed.traffic_status = status;
    CHECK(tracker.update(old_failed)); CHECK(old_failed.traffic_status == status);
    auto recovered = sample(5, .71, {person(110)}); CHECK(tracker.update(recovered));
    CHECK(recovered.obstacles[0].track_id != id);
  }
  ViewTracker tracker;
  for (uint64_t seq = 1; seq <= 2; ++seq) {
    auto both = sample(seq, seq * .1, {person(), person(100, 10)},
                       ViewId::Front, ObjectObservation::Combined); CHECK(tracker.update(both));
  }
  auto partial = sample(3, .3, {}, ViewId::Front, ObjectObservation::Combined);
  partial.traffic_status = Status::Failed; CHECK(tracker.update(partial));
  CHECK(partial.traffic_status == Status::Failed && partial.vehicle_status == Status::Valid);
  CHECK(partial.obstacles.size() == 1 && partial.obstacles[0].class_id == 10);
}
void static_images_never_predict_across_images() {
  ViewTracker tracker;
  for (uint64_t seq = 1; seq <= 2; ++seq) {
    auto still = sample(seq, seq * .1, {person()}); still.stamp.static_image = true;
    CHECK(tracker.update(still));
  }
  auto empty = sample(3, .3); empty.stamp.static_image = true;
  CHECK(tracker.update(empty)); CHECK(empty.obstacles.empty());
  auto movie = sample(4, .4); CHECK(tracker.update(movie)); CHECK(movie.obstacles.empty());
}
void sequences_and_generations_cannot_replay_tracks() {
  ViewTracker tracker; confirmed(tracker);
  auto duplicate = sample(2, .3, {person(500)}); CHECK(!tracker.update(duplicate));
  auto older = sample(1, .01); CHECK(!tracker.update(older));
  auto fresh = sample(3, .3); CHECK(tracker.update(fresh)); CHECK(fresh.obstacles.size() == 1);
  CHECK(std::abs(fresh.obstacles[0].box.left - 120) < .01f);
  auto next_generation = sample(1, .4); next_generation.stamp.generation = 2;
  CHECK(tracker.update(next_generation)); CHECK(next_generation.obstacles.empty());
  auto old_generation = sample(100, .5, {person()}); CHECK(!tracker.update(old_generation));
  auto first = sample(2, .5, {person()}); first.stamp.generation = 2; CHECK(tracker.update(first));
  auto miss = sample(3, .6); miss.stamp.generation = 2;
  CHECK(tracker.update(miss)); CHECK(miss.obstacles.empty());
}
void source_or_receive_clock_rollback_clears_motion() {
  for (bool source_rollback : {false, true}) {
    ViewTracker tracker; const auto id = confirmed(tracker);
    auto rewind = sample(3, .3, {person(110)});
    if (source_rollback) rewind.stamp.source_s = .15;
    else rewind.stamp.receive_s = .15;
    CHECK(tracker.update(rewind)); CHECK(rewind.obstacles[0].track_id != id);
    auto missing = sample(4, .31); CHECK(tracker.update(missing)); CHECK(missing.obstacles.empty());
  }
}
void duplicate_capture_time_cannot_confirm_motion() {
  ViewTracker tracker;
  auto first = sample(1, .1, {person()}); CHECK(tracker.update(first));
  auto same_time = sample(2, .1, {person()}); CHECK(tracker.update(same_time));
  CHECK(same_time.obstacles[0].track_id == first.obstacles[0].track_id);
  auto miss = sample(3, .2); CHECK(tracker.update(miss)); CHECK(miss.obstacles.empty());
}
void resolution_change_clears_coordinate_history() {
  ViewTracker tracker; const auto id = confirmed(tracker);
  auto resized = sample(3, .3, {person(110)}); resized.width = 1000;
  CHECK(tracker.update(resized)); CHECK(resized.obstacles[0].track_id != id);
  auto miss = sample(4, .4); miss.width = 1000;
  CHECK(tracker.update(miss)); CHECK(miss.obstacles.empty());
}
void legacy_ids_work_without_prediction_capability() {
  ViewTracker tracker;
  auto first = sample(1, .1, {person()}); first.width = first.height = 0;
  first.traffic_status = Status::Missing;
  auto next = first; next.stamp.sequence = 2;
  CHECK(tracker.update(first)); CHECK(tracker.update(next));
  CHECK(first.obstacles[0].track_id == next.obstacles[0].track_id);
  auto miss = sample(3, .2); CHECK(tracker.update(miss)); CHECK(miss.obstacles.empty());
  ViewTracker no_dimensions;
  first.traffic_status = next.traffic_status = Status::Valid;
  next.stamp = {1, 2, .2, .2, false};
  CHECK(no_dimensions.update(first)); CHECK(no_dimensions.update(next));
  miss = sample(3, .3); miss.width = miss.height = 0;
  CHECK(no_dimensions.update(miss)); CHECK(miss.obstacles.empty());
}
void track_and_candidate_counts_are_bounded_without_dropping_real_objects() {
  ViewTracker tracker;
  std::vector<Obstacle> crowd;
  for (int index = 0; index < 256; ++index) crowd.push_back(person(index * 50.f));
  auto first = sample(1, .1, crowd); CHECK(tracker.update(first)); CHECK(first.obstacles.size() == 256);
  auto second = sample(2, .2, crowd); CHECK(tracker.update(second)); CHECK(second.obstacles.size() == 256);
  size_t managed = 0; for (const auto &object : second.obstacles) if (object.track_id >= 0) ++managed;
  CHECK(managed <= 64);
  auto missing = sample(3, .3); CHECK(tracker.update(missing));
  CHECK(!missing.obstacles.empty() && missing.obstacles.size() <= 64);
  for (uint64_t seq = 4; seq < 100; ++seq) {
    auto many = sample(seq, seq * .1, crowd); CHECK(tracker.update(many));
    CHECK(many.obstacles.size() == 256);
    managed = 0; for (const auto &object : many.obstacles) if (object.track_id >= 0) ++managed;
    CHECK(managed <= 64);
  }
}
void invalid_numeric_input_does_not_poison_valid_tracks() {
  ViewTracker tracker; const auto id = confirmed(tracker);
  auto nan_time = sample(3, std::numeric_limits<double>::quiet_NaN());
  CHECK(!tracker.update(nan_time));
  auto invalid_view = sample(3, .3); invalid_view.view = static_cast<ViewId>(8);
  CHECK(!tracker.update(invalid_view));
  auto malformed = sample(3, .3, {person()});
  malformed.obstacles[0].box.left = std::numeric_limits<float>::quiet_NaN();
  CHECK(tracker.update(malformed));
  CHECK(malformed.obstacles.size() == 2); // Preserve the detector evidence and bounded valid track.
  CHECK(malformed.obstacles[0].track_id == -1);
  CHECK(malformed.obstacles[1].track_id == id);
  CHECK(std::isfinite(malformed.obstacles[1].box.left));
}
void metadata_uses_receive_clock_and_real_hit_count() {
  ViewTracker tracker;
  auto first = sample(1, .1, {person()}); first.stamp.receive_s = 10.1;
  auto second = sample(2, .2, {person(110)}); second.stamp.receive_s = 10.2;
  CHECK(tracker.update(first)); CHECK(tracker.update(second));
  CHECK(!second.obstacles[0].predicted);
  CHECK(second.obstacles[0].last_observed_s == 10.2);
  CHECK(second.obstacles[0].track_hits == 2);
  auto miss = sample(3, .3); miss.stamp.receive_s = 10.3;
  CHECK(tracker.update(miss)); CHECK(miss.obstacles.size() == 1);
  CHECK(miss.obstacles[0].predicted);
  CHECK(miss.obstacles[0].last_observed_s == 10.2);
  CHECK(miss.obstacles[0].track_hits == 2);
  CHECK(std::abs(miss.obstacles[0].box.left - 120.f) < .01f);
  auto real = sample(4, .4, {person(130)}); real.stamp.receive_s = 10.4;
  real.obstacles[0].track_hits = 999;
  real.obstacles[0].last_observed_s = 999;
  CHECK(tracker.update(real)); CHECK(!real.obstacles[0].predicted);
  CHECK(real.obstacles[0].track_hits == 3);
  CHECK(real.obstacles[0].last_observed_s == 10.4);
}
void predicted_input_neither_seeds_nor_refreshes_tracks() {
  ViewTracker empty;
  auto invented = person(); invented.predicted = true; invented.track_hits = 100;
  auto synthetic = sample(1, .1, {invented});
  CHECK(empty.update(synthetic)); CHECK(synthetic.obstacles.empty());
  ViewTracker tracker; const auto id = confirmed(tracker);
  auto miss = sample(3, .3); CHECK(tracker.update(miss)); CHECK(miss.obstacles.size() == 1);
  auto fed_back = sample(4, .4, miss.obstacles);
  CHECK(tracker.update(fed_back)); CHECK(fed_back.obstacles.size() == 1);
  CHECK(fed_back.obstacles[0].predicted && fed_back.obstacles[0].track_id == id);
  CHECK(fed_back.obstacles[0].last_observed_s == .2 && fed_back.obstacles[0].track_hits == 2);
  CHECK(std::abs(fed_back.obstacles[0].box.left - 130.f) < .01f);
  auto expired = sample(5, .6, fed_back.obstacles);
  CHECK(tracker.update(expired)); CHECK(expired.obstacles.empty());
}
void aggregate_initialization_and_reset_remain_compatible() {
  Obstacle legacy_five = {1, -1, {1, 2, 3, 4}, .9f, -1};
  Obstacle legacy_six = {1, -1, {1, 2, 3, 4}, .9f, -1, false};
  CHECK(!legacy_five.predicted && legacy_five.last_observed_s == 0 && legacy_five.track_hits == 0);
  CHECK(!legacy_six.predicted && legacy_six.last_observed_s == 0 && legacy_six.track_hits == 0);
  ViewTracker tracker; confirmed(tracker); tracker.reset();
  auto miss = sample(3, .3); CHECK(tracker.update(miss)); CHECK(miss.obstacles.empty());
}
void failed_measurements_and_duplicate_source_time_cannot_confirm_predictions() {
  for (bool failed_measurement : {false, true}) {
    ViewTracker tracker;
    auto first = sample(1, .1, {person()});
    auto second = sample(2, .2, {person()});
    if (failed_measurement) first.traffic_status = second.traffic_status = Status::Missing;
    else second.stamp.source_s = .1;
    CHECK(tracker.update(first)); CHECK(tracker.update(second));
    auto miss = sample(3, .3); CHECK(tracker.update(miss)); CHECK(miss.obstacles.empty());
  }
}
void unrelated_components_still_expire_old_tracks() {
  ViewTracker tracker; const auto id = confirmed(tracker);
  auto vehicle_only = sample(3, .7, {}, ViewId::Front, ObjectObservation::Vehicle);
  CHECK(tracker.update(vehicle_only)); CHECK(vehicle_only.obstacles.empty());
  auto traffic = sample(4, .71, {person(110)}); CHECK(tracker.update(traffic));
  CHECK(traffic.obstacles[0].track_id != id);
}
void duplicate_capture_time_does_not_extrapolate_a_confirmed_track() {
  ViewTracker tracker; confirmed(tracker);
  auto duplicate_capture = sample(3, .3); duplicate_capture.stamp.source_s = .2;
  CHECK(tracker.update(duplicate_capture)); CHECK(duplicate_capture.obstacles.empty());
  auto advancing = sample(4, .4); CHECK(tracker.update(advancing));
  CHECK(advancing.obstacles.size() == 1 && advancing.obstacles[0].last_observed_s == .2);
  ViewTracker repeated_image; const auto id = confirmed(repeated_image);
  auto repeated_measurement = sample(3, .5, {person(110)});
  repeated_measurement.stamp.source_s = .2;
  CHECK(repeated_image.update(repeated_measurement));
  CHECK(repeated_measurement.obstacles[0].track_id == id);
}
} // namespace

int main() {
  const std::vector<std::pair<const char *, std::function<void()>>> tests = {
    {"two_hits_predict_and_recover_motion", two_hits_predict_and_recover_motion},
    {"one_or_nonconsecutive_hits_do_not_predict", one_or_nonconsecutive_hits_do_not_predict},
    {"front_and_side_expiry_are_bounded", front_and_side_expiry_are_bounded},
    {"missed_frame_limit_applies_at_high_frame_rate", missed_frame_limit_applies_at_high_frame_rate},
    {"view_class_and_partial_components_are_isolated", view_class_and_partial_components_are_isolated},
    {"failed_or_missing_components_never_gain_predictions", failed_or_missing_components_never_gain_predictions},
    {"static_images_never_predict_across_images", static_images_never_predict_across_images},
    {"sequences_and_generations_cannot_replay_tracks", sequences_and_generations_cannot_replay_tracks},
    {"source_or_receive_clock_rollback_clears_motion", source_or_receive_clock_rollback_clears_motion},
    {"duplicate_capture_time_cannot_confirm_motion", duplicate_capture_time_cannot_confirm_motion},
    {"resolution_change_clears_coordinate_history", resolution_change_clears_coordinate_history},
    {"legacy_ids_work_without_prediction_capability", legacy_ids_work_without_prediction_capability},
    {"track_and_candidate_counts_are_bounded_without_dropping_real_objects", track_and_candidate_counts_are_bounded_without_dropping_real_objects},
    {"invalid_numeric_input_does_not_poison_valid_tracks", invalid_numeric_input_does_not_poison_valid_tracks},
    {"metadata_uses_receive_clock_and_real_hit_count", metadata_uses_receive_clock_and_real_hit_count},
    {"predicted_input_neither_seeds_nor_refreshes_tracks", predicted_input_neither_seeds_nor_refreshes_tracks},
    {"aggregate_initialization_and_reset_remain_compatible", aggregate_initialization_and_reset_remain_compatible},
    {"failed_measurements_and_duplicate_source_time_cannot_confirm_predictions", failed_measurements_and_duplicate_source_time_cannot_confirm_predictions},
    {"unrelated_components_still_expire_old_tracks", unrelated_components_still_expire_old_tracks},
    {"duplicate_capture_time_does_not_extrapolate_a_confirmed_track", duplicate_capture_time_does_not_extrapolate_a_confirmed_track},
  };
  unsigned failures = 0;
  for (const auto &test : tests) {
    try { test.second(); std::cout << "PASS " << test.first << '\n'; }
    catch (const std::exception &error) { ++failures; std::cout << "FAIL " << test.first << ": " << error.what() << '\n'; }
  }
  std::cout << tests.size() - failures << "/" << tests.size() << " tracker tests passed\n";
  return failures ? 1 : 0;
}
