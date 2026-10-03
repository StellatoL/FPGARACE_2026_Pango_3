#ifndef ADAS_ENGINE_H
#define ADAS_ENGINE_H

// ADAS 决策引擎（纯 C++，无 Qt/OpenCV 依赖，可独立单元测试）
//
// 输入：AdasPerception（推理线程产出的感知快照）+ EgoState（UI/键盘模拟的本车状态）
// 输出：AdasDecision（箭头方向、LED 掩码、报警位、停车/刹车要求、状态文案）
//
// 规则与赛题四个现场场景一一对应：
//   场景1 十字路口：红灯→stop_required；红灯+行驶→alarm_red_run；
//                   绿灯→放行；箭头随车道（左/直/右），LED0/1/2 对应指示
//   场景2 虚实线：  转向灯+实线→保持直行；强制变道+实线→alarm_illegal_lane_change；
//                   转向灯+虚线→箭头随转向（执行变道）
//   场景3 导航：    导航箭头（视频识别）覆盖默认箭头；实线时保持车道并提示
//   场景4 行人：    行人闯红灯（斑马线+红灯+行人）→brake_now 立即刹车
//   附加：超速提醒（模拟车速>限速）、碰撞提醒（TTC/近距障碍）
//
// 稳定性设计：所有易抖动的感知输入经 3-of-5 滑窗滞回（Confirm5），
// 避免单帧误检导致报警闪烁——现场演示成败的关键。

#include <deque>
#include <unordered_map>
#include <vector>

#include "core/adas_types.h"

namespace adas {

class AdasEngine {
public:
  AdasEngine() = default;

  // 执行一次决策（每收到一帧感知 / 本车状态变更时调用）
  AdasDecision step(const AdasPerception &p, const EgoState &ego);

  // 复位全部滞回状态与轨迹历史（模式切换/复位按钮用）
  void reset();

  // ---- 可调参数（标定用）----
  float ttc_threshold_s = 2.0f;   // 碰撞报警 TTC 阈值（秒）
  float min_speed_kmh = 5.0f;     // 碰撞/刹车判定的最低车速（km/h）
  float corridor_half_width_pct = 15.0f;  // 本车走廊半宽（% 帧宽）
  float near_bottom_pct = 85.0f;  // 近距判定：障碍物底边超过帧高此百分比
  float person_corridor_bottom_pct = 75.0f;  // 行人刹车走廊底边阈值

private:
  // ---- 滞回确认器 ----
  Confirm5 c_red_, c_green_, c_yellow_;          // 信号灯
  Confirm5 c_left_solid_, c_left_dashed_;        // 左线类型
  Confirm5 c_right_solid_, c_right_dashed_;      // 右线类型
  Confirm5 c_person_red_;                        // 行人闯红灯（斑马线+红灯）
  Confirm5 c_person_corridor_;                    // 行人进入本车走廊
  Confirm5 c_obstacle_near_;                      // 近距障碍

  // ---- TTC 估计：按轨迹 ID 维护框高历史 ----
  struct HeightSample {
    double t;    // 时间戳（秒）
    float h;     // 框高（像素）
  };
  std::unordered_map<int, std::deque<HeightSample>> height_hist_;

  static constexpr int kMaxSamplesPerTrack = 8;  // 每轨迹最多保留样本数
  static constexpr double kSampleWindowS = 1.0;   // 样本时间窗（秒）

  // 估计指定障碍物的 TTC（秒）；无有效增长率时返回 -1
  float estimate_ttc(const ObstacleInfo &ob, double now_s);

  // 清理超时轨迹的样本历史
  void prune_track_history(double now_s);
};

}  // namespace adas

#endif  // ADAS_ENGINE_H
