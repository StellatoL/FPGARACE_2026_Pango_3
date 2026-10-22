#ifndef TRAFFIC_SCENE_RULES_H
#define TRAFFIC_SCENE_RULES_H

#include <vector>

// 交通场景单次检测结果：包含类别 ID 和边界框
struct TrafficSceneDetection {
  int class_id = -1;          // 类别 ID（0:斑马线 1:行人 2:红灯 3:绿灯 4:黑灯 5:黄灯）
  int left = 0;               // 边界框左边界
  int top = 0;                // 边界框上边界
  int right = 0;              // 边界框右边界
  int bottom = 0;             // 边界框下边界
  float confidence = 0.0f;   // 检测置信度
};

// 交通场景分析结果：综合判断是否存在行人闯红灯等危险情况
struct TrafficSceneAnalysis {
  bool has_zebra = false;
  bool has_green_light = false;
  bool has_red_light = false;
  bool person_on_zebra = false;
  bool person_on_zebra_red = false;
  bool vehicle_on_zebra_red = false;
  TrafficSceneDetection zebra_detection;
  TrafficSceneDetection red_light_detection;
  std::vector<TrafficSceneDetection> person_detections;
};

// 判断两个检测框是否重叠（矩形相交检测）
bool traffic_rectangles_overlap(const TrafficSceneDetection &lhs,
                                const TrafficSceneDetection &rhs);

// 交通场景规则引擎：根据检测结果分析是否存在违规行为
//
// 告警逻辑：
// 1. 识别斑马线 + 绿灯同时存在
// 2. 检测到行人边界框与斑马线边界框有交集
// 3. 满足以上条件 → 判定为"绿灯时行人通过斑马线"，触发告警
TrafficSceneAnalysis analyze_traffic_scene(
    const std::vector<TrafficSceneDetection> &detections);

#endif // TRAFFIC_SCENE_RULES_H
