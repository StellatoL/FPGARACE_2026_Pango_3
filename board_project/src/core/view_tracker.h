#ifndef ADAS_VIEW_TRACKER_H
#define ADAS_VIEW_TRACKER_H

#include "core/adas_types.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace adas {
// Associations and ages belong to one view. This deliberately has no neural
// model state and can be reset on a source epoch without reloading a model.
class ViewTracker {
public:
  explicit ViewTracker(unsigned max_missed = 5, float min_iou = .3f)
      : max_missed_(max_missed), min_iou_(min_iou) {}

  bool update(ObjectObservation &observation) {
    const int index = static_cast<int>(observation.view);
    if (index < 0 || index >= 4) return false;
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
      view.has_sequence[kind] = true; view.sequence[kind] = observation.stamp.sequence; updated |= bit;
    }
    if (!updated) return false;
    observation.components = updated;
    const auto component = [](int class_id) {
      return class_id == 1 ? ObjectObservation::Traffic : class_id == 10 ? ObjectObservation::Vehicle :
             class_id == 11 ? ObjectObservation::Roadblock : 0u;
    };
    observation.obstacles.erase(std::remove_if(observation.obstacles.begin(), observation.obstacles.end(),
      [&](const Obstacle &object) { return !(updated & component(object.class_id)); }), observation.obstacles.end());
    for (auto &track : view.tracks) if (updated & component(track.class_id)) ++track.missed;

    struct Match { size_t obstacle, track; float score; };
    std::vector<Match> candidates;
    for (size_t i = 0; i < observation.obstacles.size(); ++i) {
      auto &object = observation.obstacles[i]; object.track_id = -1;
      for (size_t j = 0; j < view.tracks.size(); ++j) {
        const auto &track = view.tracks[j];
        if (object.class_id != track.class_id || track.missed > max_missed_ + 1) continue;
        const float overlap = iou(object.box, track.box);
        if (overlap >= min_iou_) candidates.push_back({i, j, overlap});
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Match &a, const Match &b) {
      return a.score > b.score;
    });
    std::vector<bool> matched(view.tracks.size(), false);
    for (const auto &match : candidates) {
      auto &object = observation.obstacles[match.obstacle];
      if (object.track_id >= 0 || matched[match.track]) continue;
      auto &track = view.tracks[match.track];
      object.track_id = track.id; track.box = object.box; track.missed = 0;
      matched[match.track] = true;
    }
    for (auto &object : observation.obstacles) {
      if (object.track_id >= 0) continue;
      const int class_slot = object.class_id == 11 ? 2 : object.class_id == 10 ? 1 : 0;
      const int64_t id = (int64_t(index + 1) << 48) |
                         (int64_t(class_slot + 1) << 40) | ++view.next_id[class_slot];
      object.track_id = id;
      view.tracks.push_back({object.class_id, id, object.box, 0});
    }
    view.tracks.erase(std::remove_if(view.tracks.begin(), view.tracks.end(),
      [&](const Track &track) { return track.missed > max_missed_; }), view.tracks.end());
    return true;
  }
  void reset() { views_ = {}; has_generation_ = false; generation_ = 0; }

private:
  struct Track { int class_id; int64_t id; Box box; unsigned missed; };
  struct View {
    std::vector<Track> tracks;
    std::array<uint64_t, 3> next_id{};
    std::array<uint64_t, 3> sequence{};
    std::array<bool, 3> has_sequence{};
  };
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
