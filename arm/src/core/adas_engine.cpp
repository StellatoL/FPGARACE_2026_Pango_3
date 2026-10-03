// ADAS 决策引擎实现
//
// step() 内部按序执行：
//   1. 感知滞回确认（灯/车道线/行人/障碍）
//   2. 场景1：信号灯 → 停车/放行/红灯强行报警；箭头随车道
//   3. 场景2：转向灯 + 车道线虚实 → 变道/保持/强制报警
//   4. 场景4：行人闯红灯 / 行人进入走廊 → 刹车
//   5. 超速提醒（模拟车速）
//   6. 碰撞提醒（TTC + 近距兜底）
//   7. 场景3：导航方向覆盖默认箭头（受实线约束）
//   8. 报警聚合：LED 全亮 + 状态文案

#include "core/adas_engine.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace adas {

void AdasEngine::reset() {
  c_red_.reset();
  c_green_.reset();
  c_yellow_.reset();
  c_left_solid_.reset();
  c_left_dashed_.reset();
  c_right_solid_.reset();
  c_right_dashed_.reset();
  c_person_red_.reset();
  c_person_corridor_.reset();
  c_obstacle_near_.reset();
  height_hist_.clear();
}

void AdasEngine::prune_track_history(double now_s) {
  for (auto it = height_hist_.begin(); it != height_hist_.end();) {
    auto &q = it->second;
    // 清理超过样本时间窗的旧样本
    while (!q.empty() && now_s - q.front().t > kSampleWindowS) {
      q.pop_front();
    }
    // 整条轨迹 2 秒无更新则删除
    if (q.empty() || now_s - q.back().t > 2.0) {
      it = height_hist_.erase(it);
    } else {
      ++it;
    }
  }
}

float AdasEngine::estimate_ttc(const ObstacleInfo &ob, double now_s) {
  if (ob.track_id < 0) {
    return -1.0f;
  }
  auto it = height_hist_.find(ob.track_id);
  if (it == height_hist_.end()) {
    return -1.0f;
  }
  auto &q = it->second;
  if (q.size() < 2) {
    return -1.0f;
  }

  const HeightSample &first = q.front();
  const HeightSample &last = q.back();
  const double dt = last.t - first.t;
  if (dt < 1e-3) {
    return -1.0f;
  }

  // 框高增长率（像素/秒）：正值 = 障碍物正在接近
  const float dh_dt = (last.h - first.h) / static_cast<float>(dt);
  if (dh_dt < 1.0f) {
    return -1.0f;  // 无明显接近
  }

  // TTC = 当前框高 / 增长率
  return last.h / dh_dt;
}

AdasDecision AdasEngine::step(const AdasPerception &p, const EgoState &ego) {
  AdasDecision d;

  // ===== 1. 感知滞回确认 =====
  const bool red_raw = p.light == LightState::Red;
  const bool green_raw = p.light == LightState::Green;
  const bool yellow_raw = p.light == LightState::Yellow;
  const bool red = c_red_.update(red_raw);
  const bool green = c_green_.update(green_raw && !red_raw);
  const bool yellow = c_yellow_.update(yellow_raw && !red_raw && !green_raw);

  // 灯优先级：红 > 黄 > 绿（同帧并存时保守）
  if (red) {
    d.light = LightState::Red;
  } else if (yellow) {
    d.light = LightState::Yellow;
  } else if (green) {
    d.light = LightState::Green;
  } else {
    d.light = LightState::Unknown;
  }

  // 车道线类型确认（左右独立）
  const bool left_solid = c_left_solid_.update(
      p.lane.left_valid && p.lane.left_type == LineType::Solid);
  const bool left_dashed = c_left_dashed_.update(
      p.lane.left_valid && p.lane.left_type == LineType::Dashed);
  const bool right_solid = c_right_solid_.update(
      p.lane.right_valid && p.lane.right_type == LineType::Solid);
  const bool right_dashed = c_right_dashed_.update(
      p.lane.right_valid && p.lane.right_type == LineType::Dashed);

  // 行人闯红灯（斑马线 + 红灯 + 行人，检测器已做空间重叠判定）
  const bool person_red = c_person_red_.update(p.person_on_zebra_red);

  // ===== 2. 场景1：信号灯决策 =====
  if (d.light == LightState::Red || d.light == LightState::Yellow) {
    d.stop_required = true;  // 红灯/黄灯 → 要求停车（黄灯保守处理）
    if (d.light == LightState::Red && ego.driving) {
      d.alarm_red_run = true;  // 红灯强行通过 → 报警
    }
  }

  // ===== 3. 箭头默认随车道（行驶行为指示）=====
  switch (ego.lane) {
    case LaneId::Left:
      d.arrow = ArrowDir::Left;
      d.led_mask = 0x1;  // LED0 = 左转
      break;
    case LaneId::Center:
      d.arrow = ArrowDir::Straight;
      d.led_mask = 0x2;  // LED1 = 直行
      break;
    case LaneId::Right:
      d.arrow = ArrowDir::Right;
      d.led_mask = 0x4;  // LED2 = 右转
      break;
  }

  // ===== 4. 场景2：虚实线变道合法性 =====
  // 变道指令 = 转向灯（意图）或强制变道（演示指令，实线时报警）
  const TurnSignal change_cmd = ego.force_lane_change != TurnSignal::None
                                    ? ego.force_lane_change
                                    : ego.signal;

  if (change_cmd == TurnSignal::Left) {
    if (left_solid) {
      // 实线：保持直行；强制指令 → 报警
      d.arrow = ArrowDir::Straight;
      d.led_mask = 0x2;
      if (ego.force_lane_change == TurnSignal::Left) {
        d.alarm_illegal_lane_change = true;
      }
    } else if (left_dashed) {
      // 虚线：允许变道，箭头随转向
      d.arrow = ArrowDir::Left;
      d.led_mask = 0x1;
    }
    // 左线未知/无左线：不授权变道（保守），保持默认箭头
  } else if (change_cmd == TurnSignal::Right) {
    if (right_solid) {
      d.arrow = ArrowDir::Straight;
      d.led_mask = 0x2;
      if (ego.force_lane_change == TurnSignal::Right) {
        d.alarm_illegal_lane_change = true;
      }
    } else if (right_dashed) {
      d.arrow = ArrowDir::Right;
      d.led_mask = 0x4;
    }
  }

  // ===== 5. 场景4：行人闯红灯 → 立即刹车 =====
  if (person_red) {
    d.brake_now = true;
    d.stop_required = true;
  }

  // 行人进入本车走廊（正前方近距）→ 刹车（不依赖灯色）
  {
    const float corridor_half =
        p.frame_w > 0 ? p.frame_w * corridor_half_width_pct / 100.0f : 0.0f;
    const float person_bottom_thr =
        p.frame_h > 0 ? p.frame_h * person_corridor_bottom_pct / 100.0f : 0.0f;
    bool person_in_corridor = false;
    for (const auto &ob : p.obstacles) {
      if (ob.class_id != 1) continue;  // 仅行人
      if (corridor_half > 0 &&
          std::abs(ob.cx - p.frame_w * 0.5f) < corridor_half &&
          ob.bottom > person_bottom_thr) {
        person_in_corridor = true;
        break;
      }
    }
    if (c_person_corridor_.update(person_in_corridor)) {
      d.brake_now = true;
      d.stop_required = true;
    }
  }

  // ===== 6. 超速提醒 =====
  if (ego.speed_kmh > ego.speed_limit + 0.5f) {
    d.alarm_overspeed = true;
  }

  // ===== 7. 碰撞提醒（TTC + 近距兜底）=====
  if (ego.speed_kmh > min_speed_kmh && p.frame_w > 0 && p.frame_h > 0) {
    const double now_s = p.timestamp_s;
    prune_track_history(now_s);

    const float corridor_half =
        p.frame_w * corridor_half_width_pct / 100.0f;
    const float near_bottom = p.frame_h * near_bottom_pct / 100.0f;

    bool obstacle_near = false;
    for (const auto &ob : p.obstacles) {
      // 更新框高历史
      if (ob.track_id >= 0) {
        auto &q = height_hist_[ob.track_id];
        const float h = static_cast<float>(ob.bottom - ob.top);
        if (q.empty() || now_s - q.back().t > 1e-3) {
          q.push_back({now_s, h});
          if (q.size() > static_cast<size_t>(kMaxSamplesPerTrack)) {
            q.pop_front();
          }
        }
      }

      // TTC 判定
      const float ttc = estimate_ttc(ob, now_s);
      if (ttc > 0.0f && ttc < ttc_threshold_s) {
        d.alarm_collision = true;
      }

      // 近距兜底：障碍物底边进入画面底部且在本车走廊内
      if (std::abs(ob.cx - p.frame_w * 0.5f) < corridor_half &&
          ob.bottom > near_bottom) {
        obstacle_near = true;
      }
    }

    if (c_obstacle_near_.update(obstacle_near)) {
      d.alarm_collision = true;
    }
  }

  // ===== 8. 场景3：导航方向覆盖（受实线约束）=====
  if (p.nav.detected) {
    const NavDir nav = p.nav.dir;
    if (nav == NavDir::Straight) {
      d.arrow = ArrowDir::Straight;
      d.led_mask = 0x2;
    } else if (nav == NavDir::Left) {
      if (left_solid) {
        // 实线：保持车道，等待虚线段（导航不构成违规，不报警）
        d.arrow = ArrowDir::Straight;
        d.led_mask = 0x2;
      } else {
        d.arrow = ArrowDir::Left;
        d.led_mask = 0x1;
      }
    } else if (nav == NavDir::Right) {
      if (right_solid) {
        d.arrow = ArrowDir::Straight;
        d.led_mask = 0x2;
      } else {
        d.arrow = ArrowDir::Right;
        d.led_mask = 0x4;
      }
    }
  }

  // ===== 9. 报警聚合与状态文案 =====
  if (d.any_alarm()) {
    d.led_mask = 0x7;  // 报警时 LED 全亮（赛题：LED 亮起模拟报警）
  }

  // 状态文案（按优先级拼接）
  {
    std::string text;
    if (d.brake_now) {
      text += u8"紧急刹车! ";
    }
    if (d.alarm_red_run) {
      text += u8"红灯强行通过! ";
    }
    if (d.alarm_illegal_lane_change) {
      text += u8"实线变道违规! ";
    }
    if (d.alarm_collision) {
      text += u8"碰撞风险! ";
    }
    if (d.alarm_overspeed) {
      text += u8"超速! ";
    }
    if (text.empty()) {
      if (d.stop_required) {
        text = ego.driving ? u8"要求停车" : u8"已停车";
      } else if (d.light == LightState::Green) {
        text = u8"绿灯通行";
      } else {
        text = u8"正常行驶";
      }
      // 附加车道线/导航提示
      if (change_cmd == TurnSignal::Left && left_solid) {
        text += u8" | 左侧实线，保持车道";
      } else if (change_cmd == TurnSignal::Right && right_solid) {
        text += u8" | 右侧实线，保持车道";
      }
      if (p.nav.detected) {
        text += u8" | 导航:";
        text += (p.nav.dir == NavDir::Left)
                    ? u8"左转"
                    : (p.nav.dir == NavDir::Right) ? u8"右转" : u8"直行";
      }
    }
    d.status_text = text;
  }

  return d;
}

}  // namespace adas
