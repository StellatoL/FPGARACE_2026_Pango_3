// 交通场景规则引擎
//   场景 A: 绿灯 + 人+斑马线 → person_on_zebra (通行提示)
//   场景 B: 红灯 + 人+斑马线 → person_on_zebra_red (行人闯红灯)
//   场景 C: 红灯 + 车辆+斑马线 → vehicle_on_zebra_red (车辆闯红灯)
// class_id: 0=zebra 1=person 2=red 3=green 4=black 5=yellow 10=vehicle

#include "core/traffic_scene_rules.h"

namespace {

bool is_valid_detection(const TrafficSceneDetection &detection) {
  return detection.right > detection.left && detection.bottom > detection.top;
}

} // namespace

bool traffic_rectangles_overlap(const TrafficSceneDetection &lhs,
                                const TrafficSceneDetection &rhs) {
  if (!is_valid_detection(lhs) || !is_valid_detection(rhs)) return false;
  return !(lhs.left > rhs.right || lhs.right < rhs.left ||
           lhs.top > rhs.bottom || lhs.bottom < rhs.top);
}

TrafficSceneAnalysis analyze_traffic_scene(
    const std::vector<TrafficSceneDetection> &detections) {
  TrafficSceneAnalysis a;

  for (const auto &d : detections) {
    if (!is_valid_detection(d)) continue;

    if (d.class_id == 0) {
      if (!a.has_zebra || d.confidence >= a.zebra_detection.confidence)
        a.zebra_detection = d;
      a.has_zebra = true;
    } else if (d.class_id == 2) {
      if (!a.has_red_light || d.confidence >= a.red_light_detection.confidence)
        a.red_light_detection = d;
      a.has_red_light = true;
    } else if (d.class_id == 3) {
      a.has_green_light = true;
    } else if (d.class_id == 1) {
      a.person_detections.push_back(d);
    }
  }

  if (!a.has_zebra) return a;

  // A: 绿灯 + 人 + 斑马线
  if (a.has_green_light) {
    for (auto &p : a.person_detections) {
      if (traffic_rectangles_overlap(p, a.zebra_detection)) {
        a.person_on_zebra = true; break;
      }
    }
  }

  // B: 红灯 + 人 + 斑马线  /  C: 红灯 + 车辆 + 斑马线
  if (a.has_red_light) {
    for (auto &p : a.person_detections) {
      if (traffic_rectangles_overlap(p, a.zebra_detection)) {
        a.person_on_zebra_red = true; break;
      }
    }
    for (const auto &d : detections) {
      if (d.class_id == 10 && traffic_rectangles_overlap(d, a.zebra_detection)) {
        a.vehicle_on_zebra_red = true; break;
      }
    }
  }

  return a;
}
