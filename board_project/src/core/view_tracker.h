#ifndef ADAS_VIEW_TRACKER_H
#define ADAS_VIEW_TRACKER_H

#include "core/adas_types.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace adas {
// Bounded constant-velocity/IoU tracker. Only real detections update its motion
// baseline; coasting boxes are evidence for retaining a hazard, not measurements.
// This borrows SORT's prediction/association idea, without its Kalman/Hungarian
// machinery or an appearance model. Identities and clocks belong to one view.
class ViewTracker {
public:
  explicit ViewTracker(unsigned max_missed = 5, float min_iou = .3f)
      : max_missed_(std::min(max_missed, 30u)),
        min_iou_(std::isfinite(min_iou) ? std::clamp(min_iou, 0.f, 1.f) : .3f) {}

  bool update(ObjectObservation &observation) {
    const int index = static_cast<int>(observation.view);
    if (index < 0 || index >= 4 || !valid_time(observation.stamp.source_s) ||
        !valid_time(observation.stamp.receive_s)) return false;
    if (has_generation_ && observation.stamp.generation < generation_) return false;
    if (!has_generation_ || observation.stamp.generation != generation_) {
      reset(); generation_ = observation.stamp.generation; has_generation_ = true;
    }
    auto &view = views_[index];
    unsigned updated = 0;
    for (int kind = 0; kind < 3; ++kind) {
      const unsigned bit = 1u << kind;
      if (!(observation.components & bit) ||
          (view.has_sequence[kind] && observation.stamp.sequence <= view.sequence[kind])) continue;
      updated |= bit;
    }
    if (!updated) return false;
    observation.components = updated;

    // A source/receive clock rewind or coordinate change starts a fresh motion
    // history for that component. Other asynchronous components keep their own
    // sequence and capture clocks. Every static image starts an independent set.
    for (int kind = 0; kind < 3; ++kind) {
      const unsigned bit = 1u << kind;
      if (!(updated & bit)) continue;
      if (view.has_sequence[kind] &&
          (observation.stamp.source_s < view.source_s[kind] ||
           observation.stamp.receive_s < view.receive_s[kind] ||
           observation.width != view.width[kind] || observation.height != view.height[kind] ||
           observation.stamp.static_image || view.static_image[kind] != observation.stamp.static_image)) {
        view.tracks.erase(std::remove_if(view.tracks.begin(), view.tracks.end(),
          [&](const Track &track) { return component(track.class_id) == bit; }), view.tracks.end());
      }
      view.has_sequence[kind] = true;
      view.sequence[kind] = observation.stamp.sequence;
      view.source_s[kind] = observation.stamp.source_s;
      view.receive_s[kind] = observation.stamp.receive_s;
      view.width[kind] = observation.width; view.height[kind] = observation.height;
      view.static_image[kind] = observation.stamp.static_image;
    }
    const double now = observation.stamp.receive_s;
    const double lifetime = index == 0 ? .35 : .75;
    // Expiration does not depend on a successful model result or on whether this
    // update includes a track's component. Failed/Missing states remain intact.
    view.tracks.erase(std::remove_if(view.tracks.begin(), view.tracks.end(),
      [&](const Track &track) { return now - track.last_observed_s > lifetime; }), view.tracks.end());

    observation.obstacles.erase(std::remove_if(observation.obstacles.begin(), observation.obstacles.end(),
      [&](const Obstacle &object) { return object.predicted || !(updated & component(object.class_id)); }),
      observation.obstacles.end());
    for (auto &object : observation.obstacles) {
      object.track_id = -1; object.predicted = false;
      object.last_observed_s = now; object.track_hits = 1;
    }
    for (auto &track : view.tracks)
      if (updated & component(track.class_id)) ++track.missed;

    struct Match { size_t obstacle, track; float score; };
    std::vector<Match> candidates;
    candidates.reserve(kMaxDetections * view.tracks.size());
    size_t considered = 0;
    for (size_t i = 0; i < observation.obstacles.size(); ++i) {
      const auto &object = observation.obstacles[i];
      if (!valid_detection(object) || considered++ >= kMaxDetections) continue;
      for (size_t j = 0; j < view.tracks.size(); ++j) {
        const auto &track = view.tracks[j];
        if (object.class_id != track.class_id) continue;
        const Box association_box = observation.stamp.source_s > track.source_s ?
                                    projected_box(track, now) : track.box;
        const float overlap = iou(object.box, association_box);
        if (overlap >= min_iou_) candidates.push_back({i, j, overlap});
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Match &a, const Match &b) {
      if (a.score != b.score) return a.score > b.score;
      return a.obstacle != b.obstacle ? a.obstacle < b.obstacle : a.track < b.track;
    });
    std::vector<bool> matched(view.tracks.size(), false);
    for (const auto &match : candidates) {
      auto &object = observation.obstacles[match.obstacle];
      if (object.track_id >= 0 || matched[match.track]) continue;
      auto &track = view.tracks[match.track];
      measure(track, object, observation);
      matched[match.track] = true;
    }
    view.tracks.erase(std::remove_if(view.tracks.begin(), view.tracks.end(),
      [&](Track &track) {
        if (track.missed) track.streak = 0;
        return track.missed > max_missed_;
      }), view.tracks.end());
    considered = 0;
    for (auto &object : observation.obstacles) {
      if (!valid_detection(object) || considered++ >= kMaxDetections || object.track_id >= 0 ||
          view.tracks.size() >= kMaxTracks) continue;
      const int class_slot = object.class_id == 11 ? 2 : object.class_id == 10 ? 1 : 0;
      if (view.next_id[class_slot] == ((uint64_t(1) << 40) - 1)) continue;
      const int64_t id = (int64_t(index + 1) << 48) |
                         (int64_t(class_slot + 1) << 40) | ++view.next_id[class_slot];
      object.track_id = id;
      Track track;
      track.class_id = object.class_id; track.id = id;
      track.box = object.box; track.confidence = object.confidence;
      track.last_observed_s = now; track.source_s = observation.stamp.source_s;
      track.width = observation.width; track.height = observation.height;
      track.eligible = eligible(observation, object.class_id);
      track.streak = track.eligible ? 1 : 0;
      view.tracks.push_back(track);
    }
    for (const auto &track : view.tracks) {
      const double age = now - track.last_observed_s;
      if (!track.missed || !(updated & component(track.class_id)) || !track.confirmed ||
          !track.eligible || !eligible(observation, track.class_id) ||
          track.width != observation.width || track.height != observation.height ||
          observation.stamp.source_s <= track.source_s ||
          age <= 0 || age > lifetime) continue;
      Box box = projected_box(track, now);
      box.left = std::clamp(box.left, 0.f, float(observation.width));
      box.right = std::clamp(box.right, 0.f, float(observation.width));
      box.top = std::clamp(box.top, 0.f, float(observation.height));
      box.bottom = std::clamp(box.bottom, 0.f, float(observation.height));
      if (!valid_box(box)) continue;
      Obstacle prediction;
      prediction.class_id = track.class_id; prediction.track_id = track.id;
      prediction.box = box;
      prediction.confidence = std::clamp(track.confidence, 0.f, 1.f) *
                              float(std::exp(-2 * age / lifetime) * std::pow(.8, track.missed));
      prediction.distance_m = -1; prediction.predicted = true;
      prediction.last_observed_s = track.last_observed_s; prediction.track_hits = track.hits;
      observation.obstacles.push_back(prediction);
    }
    return true;
  }
  void reset() { views_ = {}; has_generation_ = false; generation_ = 0; }

private:
  static constexpr size_t kMaxTracks = 64, kMaxDetections = 128;
  struct Track {
    int class_id = -1;
    int64_t id = -1;
    Box box;
    float confidence = 0, vx = 0, vy = 0;
    double last_observed_s = 0, source_s = 0;
    unsigned missed = 0, hits = 1, streak = 0;
    int width = 0, height = 0;
    bool eligible = false, confirmed = false;
  };
  struct View {
    std::vector<Track> tracks;
    std::array<uint64_t, 3> next_id{};
    std::array<uint64_t, 3> sequence{};
    std::array<bool, 3> has_sequence{};
    std::array<double, 3> source_s{}, receive_s{};
    std::array<int, 3> width{}, height{};
    std::array<bool, 3> static_image{};
  };
  static unsigned component(int class_id) {
    return class_id == 1 ? ObjectObservation::Traffic : class_id == 10 ? ObjectObservation::Vehicle :
           class_id == 11 ? ObjectObservation::Roadblock : 0u;
  }
  static bool valid_time(double value) { return std::isfinite(value) && value >= 0; }
  static bool valid_box(const Box &box) {
    return std::isfinite(box.left) && std::isfinite(box.top) &&
           std::isfinite(box.right) && std::isfinite(box.bottom) &&
           box.right > box.left && box.bottom > box.top;
  }
  static bool valid_detection(const Obstacle &object) {
    return valid_box(object.box) && std::isfinite(object.confidence) && object.confidence >= 0;
  }
  static bool eligible(const ObjectObservation &observation, int class_id) {
    const Status status = class_id == 1 ? observation.traffic_status :
                          class_id == 10 ? observation.vehicle_status : observation.roadblock_status;
    return status == Status::Valid && observation.width > 0 && observation.height > 0 &&
           !observation.stamp.static_image;
  }
  static Box projected_box(const Track &track, double now) {
    if (!track.confirmed || now <= track.last_observed_s) return track.box;
    const float dx = float(track.vx * (now - track.last_observed_s));
    const float dy = float(track.vy * (now - track.last_observed_s));
    const Box projected{track.box.left + dx, track.box.top + dy,
                        track.box.right + dx, track.box.bottom + dy};
    return valid_box(projected) ? projected : track.box;
  }
  static void measure(Track &track, Obstacle &object, const ObjectObservation &observation) {
    const double dt = observation.stamp.receive_s - track.last_observed_s;
    const bool current_eligible = eligible(observation, object.class_id);
    const bool advancing = dt > 1e-6 && observation.stamp.source_s > track.source_s;
    if (current_eligible && track.eligible && advancing) {
      const double dx = (double(object.box.left) + object.box.right - track.box.left - track.box.right) * .5;
      const double dy = (double(object.box.top) + object.box.bottom - track.box.top - track.box.bottom) * .5;
      track.vx = float(std::clamp(dx / dt, -4. * observation.width, 4. * observation.width));
      track.vy = float(std::clamp(dy / dt, -4. * observation.height, 4. * observation.height));
      track.streak = track.missed == 1 ? increment(track.streak) : 1;
      track.confirmed = track.confirmed || track.streak >= 2;
    } else {
      track.vx = track.vy = 0;
      track.streak = current_eligible ? 1 : 0;
      track.confirmed = false;
    }
    track.box = object.box; track.confidence = object.confidence;
    track.last_observed_s = observation.stamp.receive_s; track.source_s = observation.stamp.source_s;
    track.width = observation.width; track.height = observation.height;
    track.eligible = current_eligible; track.missed = 0; track.hits = increment(track.hits);
    object.track_id = track.id; object.track_hits = track.hits;
  }
  static unsigned increment(unsigned value) {
    return value < std::numeric_limits<unsigned>::max() ? value + 1 : value;
  }
  static float iou(const Box &a, const Box &b) {
    const float intersection = std::max(0.f, std::min(a.right, b.right) - std::max(a.left, b.left)) *
                               std::max(0.f, std::min(a.bottom, b.bottom) - std::max(a.top, b.top));
    const float total = std::max(0.f, a.right-a.left) * std::max(0.f, a.bottom-a.top) +
                        std::max(0.f, b.right-b.left) * std::max(0.f, b.bottom-b.top) - intersection;
    return std::isfinite(total) && total > 0 ? intersection / total : 0;
  }
  std::array<View, 4> views_;
  uint64_t generation_ = 0;
  bool has_generation_ = false;
  unsigned max_missed_;
  float min_iou_;
};

// Preserve each model's capture clock when geometry consumes asynchronous
// person and vehicle observations. Updating one component never refreshes the
// other component's masks.
class FrontObjectCache {
public:
  bool update(const ObjectObservation &observation) {
    if (observation.view != ViewId::Front ||
        (has_generation_ && observation.stamp.generation < generation_)) return false;
    if (!has_generation_ || observation.stamp.generation != generation_) {
      clear(); generation_ = observation.stamp.generation; has_generation_ = true;
    }
    bool accepted = false;
    for (int kind = 0; kind < 2; ++kind) {
      if (!(observation.components & (1u << kind)) ||
          (present_[kind] && observation.stamp.sequence <= observations_[kind].stamp.sequence)) continue;
      observations_[kind] = observation; present_[kind] = true; accepted = true;
    }
    return accepted;
  }
  // Person masks expire at 250 ms; the slow optional vehicle model gets 750 ms
  // from capture, rather than a fresh timestamp at its inference completion.
  std::vector<Box> boxes(const Stamp &stamp, int width, int height,
                         double traffic_age_s = .25, double vehicle_age_s = .75) const {
    std::vector<Box> result;
    for (int kind = 0; kind < 2; ++kind) {
      const auto &observation = observations_[kind];
      const double age = stamp.source_s - observation.stamp.source_s;
      const Status status = kind == 0 ? observation.traffic_status : observation.vehicle_status;
      const double max_age_s = kind == 0 ? traffic_age_s : vehicle_age_s;
      if (!present_[kind] || status != Status::Valid || stamp.generation != observation.stamp.generation ||
          observation.stamp.sequence > stamp.sequence || observation.width != width || observation.height != height ||
          !std::isfinite(age) || age < 0 || age > max_age_s) continue;
      for (const auto &object : observation.obstacles)
        if (object.class_id == (kind == 0 ? 1 : 10)) result.push_back(object.box);
    }
    return result;
  }
  void clear() { observations_ = {}; present_ = {}; generation_ = 0; has_generation_ = false; }
private:
  std::array<ObjectObservation, 2> observations_;
  std::array<bool, 2> present_{};
  uint64_t generation_ = 0;
  bool has_generation_ = false;
};
} // namespace adas
#endif
