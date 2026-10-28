#ifndef ADAS_TYPES_H
#define ADAS_TYPES_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace adas {
enum class ViewId : int { Front = 0, Right = 1, Rear = 2, Left = 3 };
enum class Layout : int { Mosaic = 0, Single = 1 };
enum class Status : int { Missing = 0, Valid, Failed };
enum class LaneId : int { Left = 0, Center = 1, Right = 2 };
enum class TurnSignal : int { None = 0, Left, Right };
enum class Direction : int { None = 0, Left, Straight, Right };
using NavDir = Direction;
using ArrowDir = Direction;
enum class LineType : int { Unknown = 0, Solid, Dashed };
enum class LightState : int { Unknown = 0, Red, Green, Yellow, Black };
enum class SpeedSource : int { Simulation = 0, External, Unavailable };
enum class Maneuver : int { KeepLane = 0, ChangeLeft, ChangeRight, Stop };

struct Stamp {
  uint64_t generation = 0, sequence = 0;
  double source_s = 0, receive_s = 0;
  bool static_image = false;
};
struct Roi { double x = 0, y = 0, w = 1, h = 1; };
struct Box { float left = 0, top = 0, right = 0, bottom = 0; };
struct LaneInfo {
  bool left_valid = false, right_valid = false;
  LineType left_type = LineType::Unknown, right_type = LineType::Unknown;
  float left_x = 0, right_x = 0, center_offset = 0;
  float left_top_x = 0, right_top_x = 0, top_y = 0, bottom_y = 0;
  float left_conf = 0, right_conf = 0;
};
struct NavInfo {
  bool detected = false;
  Direction dir = Direction::None;
  float confidence = 0;
  Box box;
};
struct SpeedResult {
  bool valid = false;
  float speed_kmh = 0;
  int features = 0;
  std::string reason = "未标定";
};
struct Obstacle {
  int class_id = -1; // 1=person, 10=vehicle, 11=roadblock adapter; others ignored.
  int64_t track_id = -1;
  Box box;
  float confidence = 0;
  float distance_m = -1;
  bool predicted = false;
  double last_observed_s = 0; // Monotonic receive clock of the last real measurement.
  unsigned track_hits = 0; // Real measurements only; predictions never add hits.
};
struct ObjectObservation {
  // A partial observation updates only the listed model components. Cached
  // results from a different model must never acquire this sample's timestamp.
  static constexpr unsigned Traffic = 1, Vehicle = 2, Roadblock = 4, Signal = 8,
                            Crosswalk = 16, Combined = 3, All = 7;
  unsigned components = Combined;
  Stamp stamp;
  ViewId view = ViewId::Front;
  Status traffic_status = Status::Missing;
  Status vehicle_status = Status::Missing;
  Status roadblock_status = Status::Missing;
  int width = 0, height = 0;
  LightState light = LightState::Unknown;
  bool has_lane_lights = false;
  std::array<LightState, 3> lane_lights{};
  bool has_zebra = false, person_on_zebra = false;
  std::vector<Obstacle> obstacles;
  float infer_ms = 0;
  std::string error;
  Status signal_status = Status::Missing;
  bool signal_observation = true; // Legacy full-scene light producer; neural ROI owns signals separately.
  Status crosswalk_status = Status::Missing;
};
struct SignalObservation {
  Stamp stamp;
  Status status = Status::Missing;
  LightState light = LightState::Unknown;
  bool has_lane_lights = false;
  std::array<LightState,3> lane_lights{};
};
struct GeometryObservation {
  static constexpr unsigned Lane = 1, Navigation = 2, Speed = 4, All = 7;
  unsigned components = All;
  Stamp stamp;
  Status status = Status::Missing;
  int width = 0, height = 0;
  LaneInfo lane;
  NavInfo nav;
  SpeedResult speed;
  float process_ms = 0;
};
struct LaneObservation {
  Stamp stamp;
  Status status = Status::Missing;
  int width = 0, height = 0;
  LaneInfo lane;
  float process_ms = 0;
};
struct NavigationObservation {
  Stamp stamp;
  Status status = Status::Missing;
  int width = 0, height = 0;
  NavInfo nav;
};
struct SpeedSample {
  Stamp stamp;
  Status status = Status::Missing;
  SpeedSource source = SpeedSource::Unavailable;
  float speed_kmh = 0;
  std::string reason;
};
struct PathPoint { float x = .5f, y = 1; };
struct LocalPlan {
  bool valid = false;
  Maneuver maneuver = Maneuver::Stop;
  std::array<PathPoint,16> points{};
  unsigned point_count = 0;
  std::string reason;
};
struct EgoState {
  Layout input_layout = Layout::Mosaic;
  LaneId lane = LaneId::Center;
  bool automatic = true, manual_stop = false, force_drive = false;
  bool paused = false, intersection_mode = false;
  TurnSignal signal = TurnSignal::None;
  TurnSignal force_lane_change = TurnSignal::None;
  bool manual_nav_enabled = false;
  Direction manual_nav = Direction::Straight;
  // Kept for old diagnostic callers; production decisions use speed_source.
  bool use_visual_speed = false;
  SpeedSource speed_source = SpeedSource::Simulation;
  // Monotonic receive-clock time of the active turn/navigation request. Zero
  // preserves standalone diagnostic compatibility; the UI supplies real times.
  double lane_change_request_s = 0;
  float manual_speed_kmh = 0, speed_limit = 60;
};
struct AdasDecision {
  bool stop_required = false, brake_now = false, auto_driving = false;
  bool alarm_red_run = false, alarm_illegal_lane_change = false;
  bool alarm_overspeed = false, alarm_collision = false;
  bool alarm_lane_departure = false, lane_change_allowed = false;
  bool speed_valid = false;
  SpeedSource speed_source = SpeedSource::Unavailable;
  float speed_kmh = 0, nearest_obstacle_m = -1, ttc_s = -1;
  Direction arrow = Direction::Straight;
  LightState light = LightState::Unknown;
  uint8_t led_mask = 2;
  std::string status_text, hint_text;
  LocalPlan local_plan;
  bool any_alarm() const {
    return brake_now || alarm_red_run || alarm_illegal_lane_change ||
           alarm_overspeed || alarm_collision || alarm_lane_departure;
  }
};
inline const char *line_text(LineType v) {
  return v == LineType::Solid ? u8"实线" : v == LineType::Dashed ? u8"虚线" : u8"未知";
}
inline const char *light_text(LightState v) {
  switch (v) {
  case LightState::Red: return u8"红灯";
  case LightState::Green: return u8"绿灯";
  case LightState::Yellow: return u8"黄灯";
  default: return u8"无信号";
  }
}
inline double monotonic_seconds(); // Defined in common/frame_packet.h.
} // namespace adas
#endif
