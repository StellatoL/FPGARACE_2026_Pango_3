#ifndef ADAS_TYPES_H
#define ADAS_TYPES_H

// ADAS 公共类型定义 —— 纯 C++17，无 Qt/OpenCV 依赖，可独立单元测试。
// 与赛题（选题三）四个现场场景一一对应：
//   场景1 十字路口红绿灯 / 场景2 虚实线变道
//   场景3 导航模拟智驾   / 场景4 行人闯红灯刹车

#include <cstdint>
#include <string>
#include <vector>

namespace adas {

// ===== 枚举 =====

// 本车所在车道（按键切换：1/2/3 或 UI 单选）
enum class LaneId : uint8_t { Left = 0, Center = 1, Right = 2 };

// 转向灯（按键模拟：A/D）
enum class TurnSignal : uint8_t { None = 0, Left = 1, Right = 2 };

// 信号灯状态（NPU 检测 6 类中的 2红/3绿/4黑/5黄 归并）
enum class LightState : uint8_t { Unknown = 0, Red, Green, Yellow, Black };

// 行驶行为箭头方向（叠加显示 / LED 指示）
enum class ArrowDir : uint8_t { None = 0, Left, Straight, Right };

// 车道线类型（场景2 判决依据）
enum class LineType : uint8_t { Unknown = 0, Solid, Dashed };

// 导航指令方向（场景3：识别视频内导航箭头）
enum class NavDir : uint8_t { None = 0, Left, Straight, Right };

// ===== 感知结构（推理线程产出，随 VisionFrameResult 传递）=====

// 车道线虚实分析结果（LaneLineAnalyzer 每帧产出，无帧间状态）
struct LaneLineInfo {
  bool left_valid = false;             // 左线是否有效检出
  bool right_valid = false;            // 右线是否有效检出
  LineType left_type = LineType::Unknown;  // 左线类型
  LineType right_type = LineType::Unknown; // 右线类型
  int left_x = 0;                       // 左线在 ROI 底部的 x 坐标
  int right_x = 0;                      // 右线在 ROI 底部的 x 坐标
  float left_conf = 0.0f;               // 左线置信度 [0,1]
  float right_conf = 0.0f;              // 右线置信度 [0,1]
  int lane_center_offset = 0;           // 车道中心相对画面中心的偏移（像素）
};

// 导航箭头识别结果（NavArrowDetector 产出，含帧间确认状态）
struct NavArrowInfo {
  bool detected = false;                // 当前是否有已确认的导航方向
  NavDir dir = NavDir::None;            // 导航方向
  float conf = 0.0f;                    // 判向置信度 [0,1]
  int cx = 0;                           // 箭头中心 x（叠加框提示用）
  int cy = 0;                           // 箭头中心 y
};

// 单个障碍物（SORT 跟踪输出：行人/车辆）
struct ObstacleInfo {
  int class_id = -1;                    // 1=行人 10=车辆（与 traffic_scene_rules 一致）
  int track_id = -1;                    // SORT 轨迹 ID（TTC 估计用）
  float cx = 0.0f, cy = 0.0f;           // 中心坐标
  float vx = 0.0f, vy = 0.0f;           // 速度（像素/帧）
  int left = 0, top = 0, right = 0, bottom = 0;  // 边界框
};

// 一帧 ADAS 感知快照：NPU 检测 + 车道线 + 导航箭头的聚合
struct AdasPerception {
  bool valid = false;                   // 本帧感知是否有效
  LightState light = LightState::Unknown;  // 信号灯状态（原始，未经滞回）
  int light_cx = 0, light_cy = 0;       // 信号灯框中心
  bool has_zebra = false;               // 检出斑马线
  bool person_on_zebra = false;         // 行人在斑马线上（绿灯）
  bool person_on_zebra_red = false;    // 行人在斑马线上（红灯）→ 场景4 刹车
  std::vector<ObstacleInfo> obstacles;  // 跟踪中的障碍物
  LaneLineInfo lane;                    // 车道线虚实
  NavArrowInfo nav;                     // 导航箭头
  float infer_ms = 0.0f;                // 本帧推理耗时（HUD 显示）
  double timestamp_s = 0.0;             // 单调时间戳（秒，TTC 计算用）
  int frame_w = 0, frame_h = 0;         // 帧尺寸（走廊/近距判定用）
};

// 3-of-5 滑窗滞回确认器：连续 5 次观测中 ≥3 次为真才置位。
// 用于抑制单帧抖动导致的报警闪烁（演示稳定性的关键）。
struct Confirm5 {
  uint8_t hist = 0;  // 5 bit 滑窗历史（最新位在低位）

  bool update(bool raw) {
    hist = static_cast<uint8_t>(((hist << 1) | (raw ? 1 : 0)) & 0x1F);
    int n = 0;
    for (int i = 0; i < 5; ++i)
      if (hist & (1 << i)) ++n;
    return n >= 3;
  }

  void reset() { hist = 0; }
};

// ===== 本车模拟状态（UI/键盘驱动）=====

struct EgoState {
  LaneId lane = LaneId::Center;         // 当前车道
  bool driving = false;                 // 行驶中 / 已停车（红灯时置 true 即"强制行驶"）
  TurnSignal signal = TurnSignal::None; // 转向灯（变道意图）
  float speed_kmh = 0.0f;               // 模拟车速（滑块）
  float speed_limit = 60.0f;            // 限速（超速提醒阈值）
  TurnSignal force_lane_change = TurnSignal::None; // 演示：强制变道指令（实线时触发报警）
};

// ===== 决策输出（AdasEngine 产出，渲染/报警依据）=====

struct AdasDecision {
  bool stop_required = false;           // 要求停车（红灯/行人/刹车）
  bool brake_now = false;               // 立即刹车（场景4 行人闯红灯）
  bool alarm_red_run = false;           // 报警：红灯强行通过（场景1）
  bool alarm_illegal_lane_change = false; // 报警：实线变道（场景2）
  bool alarm_overspeed = false;         // 报警：超速
  bool alarm_collision = false;         // 报警：碰撞风险（TTC/近距）
  ArrowDir arrow = ArrowDir::None;      // 行驶行为箭头（叠加+LED）
  uint8_t led_mask = 0;                 // LED 位掩码：b0=LED0左 b1=LED1直 b2=LED2右
  LightState light = LightState::Unknown; // 滞回确认后的信号灯状态
  std::string status_text;              // 状态栏/HUD 文案

  bool any_alarm() const {
    return alarm_red_run || alarm_illegal_lane_change || alarm_overspeed ||
           alarm_collision || brake_now;
  }
};

// ===== 枚举转文字（UI 显示用，UTF-8 中文）=====

inline const char *lane_id_text(LaneId lane) {
  switch (lane) {
    case LaneId::Left:   return u8"左车道";
    case LaneId::Right:  return u8"右车道";
    case LaneId::Center: return u8"直行车道";
  }
  return u8"未知";
}

inline const char *light_state_text(LightState light) {
  switch (light) {
    case LightState::Red:    return u8"红灯";
    case LightState::Green:  return u8"绿灯";
    case LightState::Yellow: return u8"黄灯";
    case LightState::Black:  return u8"黑灯";
    case LightState::Unknown: return u8"无信号";
  }
  return u8"无信号";
}

inline const char *line_type_text(LineType type) {
  switch (type) {
    case LineType::Solid:   return u8"实线";
    case LineType::Dashed: return u8"虚线";
    case LineType::Unknown: return u8"未知";
  }
  return u8"未知";
}

inline const char *arrow_dir_text(ArrowDir dir) {
  switch (dir) {
    case ArrowDir::Left:     return u8"左转";
    case ArrowDir::Straight: return u8"直行";
    case ArrowDir::Right:    return u8"右转";
    case ArrowDir::None:     return u8"无";
  }
  return u8"无";
}

}  // namespace adas

#endif  // ADAS_TYPES_H
