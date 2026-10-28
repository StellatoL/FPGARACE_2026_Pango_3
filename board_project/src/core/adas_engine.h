#ifndef ADAS_ENGINE_H
#define ADAS_ENGINE_H
#include "core/adas_types.h"
#include <deque>
#include <map>
#include <utility>

namespace adas {
class AdasEngine {
public:
  void reset(uint64_t generation = 0);
  // Standalone callers may use the observation clock; asynchronous consumers
  // pass their actual monotonic time so delayed samples cannot confirm releases.
  bool ingest(const ObjectObservation &observation, double now_s = -1);
  bool ingest(const SignalObservation &observation, double now_s = -1);
  bool ingest(const GeometryObservation &observation, double now_s = -1);
  bool ingest(const LaneObservation &observation, double now_s = -1);
  bool ingest(const NavigationObservation &observation, double now_s = -1);
  bool ingest(const SpeedSample &sample, double now_s = -1);
  AdasDecision evaluate(const EgoState &ego, double now_s) const;
  uint64_t generation() const { return generation_; }
  double front_timeout_s = .5, side_timeout_s = 1.5, external_speed_timeout_s = 1;
  float ttc_threshold_s = 2;
  // The default profile requires all legacy obstacle sources. Only an explicit
  // person/vehicle profile may leave Roadblock unconfigured.
  unsigned required_obstacle_components = ObjectObservation::All;
  bool separate_crosswalk_source = false;
private:
  struct Confirm {
    unsigned bits = 0;
    bool update(bool raw, bool still = false) {
      bits = ((bits << 1) | unsigned(raw)) & 31;
      unsigned b = bits; int count = 0;
      while (b) { count += b & 1; b >>= 1; }
      return still ? raw : count >= 3;
    }
  };
  bool accept(const Stamp &stamp, const Stamp &old);
  static bool fresh(const Stamp &stamp, double now, double timeout);
  bool obstacle_sources_ready(int view, double now, double request_s = 0) const;
  void update_risk(const ObjectObservation &observation, unsigned component);
  std::array<ObjectObservation,4> objects_{};
  std::array<Stamp,4> traffic_stamps_{}, vehicle_stamps_{}, roadblock_stamps_{};
  std::array<Stamp,4> crosswalk_stamps_{};
  LaneObservation lane_;
  NavigationObservation navigation_;
  SpeedSample speed_;
  Stamp legacy_speed_stamp_;
  Stamp signal_stamp_;
  Status signal_status_ = Status::Missing;
  std::array<bool,3> junction_context_{}, green_ready_{};
  std::array<std::array<Confirm,3>,3> light_confirm_{};
  std::array<LightState,3> lights_{};
  std::array<Confirm,3> no_signal_confirm_{};
  std::array<Confirm,3> nav_confirm_{};
  Direction confirmed_nav_ = Direction::None;
  Confirm left_solid_, left_dashed_, right_solid_, right_dashed_;
  bool left_solid = false, left_dashed = false, right_solid = false, right_dashed = false;
  Confirm danger_confirm_;
  bool emergency_ = false;
  int clear_count_ = 0;
  Confirm crosswalk_danger_confirm_;
  bool crosswalk_emergency_ = false;
  int crosswalk_clear_count_ = 0;
  struct Risk { bool latched = false; int clear_count = 0; float nearest = -1, ttc = -1; };
  std::array<Risk,3> risks_{};
  struct Sample { double t; float h, z; };
  std::map<std::pair<int,int64_t>,std::deque<Sample>> history_;
  uint64_t generation_ = 0;
};
} // namespace adas
#endif
