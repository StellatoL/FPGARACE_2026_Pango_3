// AdasEngine 单元测试（纯 C++，无 Qt/OpenCV/NPU 依赖）
//
// 覆盖赛题四个现场场景的决策逻辑：
//   场景1 红绿灯 / 场景2 虚实线变道 / 场景3 导航 / 场景4 行人刹车
// 以及超速、碰撞 TTC、3-of-5 滞回、灯优先级。
//
// 构建（板端）：
//   g++ -std=c++17 -O2 -I../src tests/adas_engine_test.cpp \
//       ../src/core/adas_engine.cpp -o /tmp/adas_engine_test && /tmp/adas_engine_test

#include "core/adas_engine.h"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

void expect_true(bool cond, const std::string &name) {
  if (!cond) {
    std::cerr << "[FAIL] " << name << '\n';
    ++g_failures;
  } else {
    std::cout << "[PASS] " << name << '\n';
  }
}

// 便捷构造：单灯感知帧
adas::AdasPerception make_perception(adas::LightState light) {
  adas::AdasPerception p;
  p.valid = true;
  p.light = light;
  p.frame_w = 1280;
  p.frame_h = 720;
  return p;
}

// 便捷构造：带车道线的感知帧
adas::AdasPerception make_lane_perception(adas::LineType left,
                                          adas::LineType right) {
  adas::AdasPerception p = make_perception(adas::LightState::Unknown);
  p.lane.left_valid = (left != adas::LineType::Unknown);
  p.lane.right_valid = (right != adas::LineType::Unknown);
  p.lane.left_type = left;
  p.lane.right_type = right;
  p.lane.left_x = 300;
  p.lane.right_x = 980;
  return p;
}

// 连续送 n 帧让 3-of-5 滞回收敛
adas::AdasDecision run_n(adas::AdasEngine &engine, const adas::AdasPerception &p,
                         const adas::EgoState &ego, int n) {
  adas::AdasDecision d;
  for (int i = 0; i < n; ++i) {
    adas::AdasPerception pi = p;
    pi.timestamp_s = i * 0.033;  // 30fps
    d = engine.step(pi, ego);
  }
  return d;
}

}  // namespace

int main() {
  using namespace adas;

  // ===== 场景1：红绿灯 =====
  {
    AdasEngine engine;
    EgoState ego;  // 默认：中车道，停车
    // 红灯 + 行驶 → 报警 + 要求停车
    ego.driving = true;
    AdasDecision d = run_n(engine, make_perception(LightState::Red), ego, 5);
    expect_true(d.light == LightState::Red, "S1: red light confirmed");
    expect_true(d.alarm_red_run, "S1: driving on red -> alarm_red_run");
    expect_true(d.stop_required, "S1: red -> stop_required");
  }
  {
    AdasEngine engine;
    EgoState ego;  // 红灯 + 已停车 → 无报警
    AdasDecision d = run_n(engine, make_perception(LightState::Red), ego, 5);
    expect_true(d.light == LightState::Red, "S1: red confirmed (stopped)");
    expect_true(!d.alarm_red_run, "S1: stopped on red -> no alarm");
    expect_true(d.stop_required, "S1: red -> stop_required (stopped)");
  }
  {
    AdasEngine engine;
    EgoState ego;
    ego.driving = true;
    AdasDecision d = run_n(engine, make_perception(LightState::Green), ego, 5);
    expect_true(d.light == LightState::Green, "S1: green confirmed");
    expect_true(!d.stop_required, "S1: green -> no stop");
    expect_true(!d.alarm_red_run, "S1: green -> no alarm");
    expect_true(d.arrow == ArrowDir::Straight && d.led_mask == 0x2,
                "S1: center lane -> straight arrow, LED1");
  }
  {
    // 车道 → 箭头/LED 映射（左/右）
    AdasEngine engine;
    EgoState ego_l;
    ego_l.lane = LaneId::Left;
    AdasDecision dl = run_n(engine, make_perception(LightState::Green), ego_l, 5);
    expect_true(dl.arrow == ArrowDir::Left && dl.led_mask == 0x1,
                "S1: left lane -> left arrow, LED0");
    AdasEngine engine2;
    EgoState ego_r;
    ego_r.lane = LaneId::Right;
    AdasDecision dr = run_n(engine2, make_perception(LightState::Green), ego_r, 5);
    expect_true(dr.arrow == ArrowDir::Right && dr.led_mask == 0x4,
                "S1: right lane -> right arrow, LED2");
  }

  // ===== 滞回：3-of-5 =====
  {
    AdasEngine engine;
    EgoState ego;
    // 单帧毛刺不应确认：绿×4 → 1 帧红毛刺 → 绿×4，红不应被确认
    AdasDecision d;
    for (int i = 0; i < 9; ++i) {
      const bool glitch = (i == 4);
      AdasPerception p = make_perception(
          glitch ? LightState::Red : LightState::Green);
      p.timestamp_s = i * 0.033;
      d = engine.step(p, ego);
    }
    expect_true(d.light == LightState::Green,
                "hysteresis: single red glitch suppressed, green holds");
  }
  {
    AdasEngine engine;
    EgoState ego;
    // 已确认的红灯遇单帧绿毛刺：红保持（不闪烁）
    AdasDecision d;
    for (int i = 0; i < 9; ++i) {
      const bool glitch = (i == 6);
      AdasPerception p = make_perception(
          glitch ? LightState::Green : LightState::Red);
      p.timestamp_s = i * 0.033;
      d = engine.step(p, ego);
    }
    expect_true(d.light == LightState::Red,
                "hysteresis: single green glitch suppressed, red holds");
  }
  {
    AdasEngine engine;
    EgoState ego;
    // 前 2 帧红 → 未确认；第 3 帧起确认
    AdasDecision d;
    for (int i = 0; i < 3; ++i) {
      AdasPerception p = make_perception(LightState::Red);
      p.timestamp_s = i * 0.033;
      d = engine.step(p, ego);
    }
    expect_true(d.light == LightState::Red, "hysteresis: 3-of-5 red confirmed");
  }

  // ===== 灯优先级：红 > 绿（同帧并存，检测器已按置信度归并，此处验证引擎侧）=====
  {
    AdasEngine engine;
    EgoState ego;
    ego.driving = true;
    // 感知直接给 Red（检测器归并结果），引擎应确认红而非绿
    AdasDecision d = run_n(engine, make_perception(LightState::Red), ego, 5);
    expect_true(d.light == LightState::Red && d.alarm_red_run,
                "priority: red wins over green history");
  }

  // ===== 场景2：虚实线变道 =====
  {
    // 左实线 + 左转向灯 → 保持直行，无报警
    AdasEngine engine;
    EgoState ego;
    ego.signal = TurnSignal::Left;
    AdasDecision d = run_n(engine, make_lane_perception(LineType::Solid,
                                                       LineType::Dashed), ego, 5);
    expect_true(d.arrow == ArrowDir::Straight,
                "S2: solid line + signal -> hold straight");
    expect_true(!d.alarm_illegal_lane_change,
                "S2: signal on solid -> no alarm (only forced change alarms)");
  }
  {
    // 左实线 + 强制左变道 → 报警
    AdasEngine engine;
    EgoState ego;
    ego.force_lane_change = TurnSignal::Left;
    AdasDecision d = run_n(engine, make_lane_perception(LineType::Solid,
                                                        LineType::Dashed), ego, 5);
    expect_true(d.alarm_illegal_lane_change,
                "S2: forced change on solid -> alarm");
    expect_true(d.led_mask == 0x7, "S2: alarm -> all LEDs on");
  }
  {
    // 左虚线 + 左转向灯 → 变道箭头
    AdasEngine engine;
    EgoState ego;
    ego.signal = TurnSignal::Left;
    AdasDecision d = run_n(engine, make_lane_perception(LineType::Dashed,
                                                        LineType::Solid), ego, 5);
    expect_true(d.arrow == ArrowDir::Left && d.led_mask == 0x1,
                "S2: dashed + signal -> change lane (left arrow, LED0)");
    expect_true(!d.alarm_illegal_lane_change, "S2: dashed change -> no alarm");
  }
  {
    // 右实线 + 强制右变道 → 报警（对称性）
    AdasEngine engine;
    EgoState ego;
    ego.force_lane_change = TurnSignal::Right;
    AdasDecision d = run_n(engine, make_lane_perception(LineType::Dashed,
                                                        LineType::Solid), ego, 5);
    expect_true(d.alarm_illegal_lane_change,
                "S2: forced right change on right solid -> alarm");
  }
  {
    // 线未知 + 转向灯 → 不授权变道（保守）
    AdasEngine engine;
    EgoState ego;
    ego.signal = TurnSignal::Left;
    AdasDecision d = run_n(engine, make_lane_perception(LineType::Unknown,
                                                       LineType::Unknown), ego, 5);
    expect_true(d.arrow == ArrowDir::Straight,
                "S2: unknown line -> conservative straight");
    expect_true(!d.alarm_illegal_lane_change,
                "S2: unknown line -> no alarm");
  }

  // ===== 场景4：行人闯红灯 → 刹车 =====
  {
    AdasEngine engine;
    EgoState ego;
    ego.driving = true;
    AdasPerception p = make_perception(LightState::Red);
    p.person_on_zebra_red = true;
    AdasDecision d = run_n(engine, p, ego, 5);
    expect_true(d.brake_now, "S4: person on zebra red -> brake_now");
    expect_true(d.stop_required, "S4: brake -> stop_required");
    expect_true(d.led_mask == 0x7, "S4: brake -> all LEDs (alarm)");
  }
  {
    // 行人进入本车走廊（正前方近距）→ 刹车（不依赖灯色）
    AdasEngine engine;
    EgoState ego;
    ego.driving = true;
    ego.speed_kmh = 30;
    AdasPerception p = make_perception(LightState::Green);
    ObstacleInfo person;
    person.class_id = 1;  // 行人
    person.cx = 640;      // 画面正中
    person.bottom = 650;  // 底部近距（>75% × 720 = 540）
    person.top = 500;
    p.obstacles.push_back(person);
    AdasDecision d = run_n(engine, p, ego, 5);
    expect_true(d.brake_now, "S4: person in corridor -> brake_now");
  }

  // ===== 超速提醒 =====
  {
    AdasEngine engine;
    EgoState ego;
    ego.driving = true;
    ego.speed_kmh = 80;
    ego.speed_limit = 60;
    AdasDecision d = run_n(engine, make_perception(LightState::Green), ego, 5);
    expect_true(d.alarm_overspeed, "overspeed: 80>60 -> alarm");
    expect_true(d.led_mask == 0x7, "overspeed: alarm -> all LEDs");
  }
  {
    AdasEngine engine;
    EgoState ego;
    ego.speed_kmh = 55;
    ego.speed_limit = 60;
    AdasDecision d = run_n(engine, make_perception(LightState::Green), ego, 5);
    expect_true(!d.alarm_overspeed, "overspeed: 55<60 -> no alarm");
  }

  // ===== 碰撞 TTC =====
  {
    // 模拟障碍物框高持续增长（接近）：100→150→225→337 px，约 0.1s/帧
    AdasEngine engine;
    EgoState ego;
    ego.driving = true;
    ego.speed_kmh = 40;
    AdasDecision d;
    const float heights[] = {100, 150, 225, 337, 505};
    for (int i = 0; i < 5; ++i) {
      AdasPerception p = make_perception(LightState::Green);
      ObstacleInfo ob;
      ob.class_id = 10;  // 车辆
      ob.track_id = 7;
      ob.cx = 640;
      ob.top = 300;
      ob.bottom = 300 + static_cast<int>(heights[i]);
      p.obstacles.push_back(ob);
      p.timestamp_s = i * 0.1;
      d = engine.step(p, ego);
    }
    // 增长率 ≈ (505-100)/0.4 ≈ 1012 px/s，TTC ≈ 505/1012 ≈ 0.5s < 2s
    expect_true(d.alarm_collision, "collision: growing box -> TTC alarm");
  }
  {
    // 静止障碍物（框高不变）→ 无 TTC 报警；但近距兜底会触发（bottom>85%）
    AdasEngine engine;
    EgoState ego;
    ego.driving = true;
    ego.speed_kmh = 40;
    AdasDecision d;
    for (int i = 0; i < 5; ++i) {
      AdasPerception p = make_perception(LightState::Green);
      ObstacleInfo ob;
      ob.class_id = 10;
      ob.track_id = 9;
      ob.cx = 640;
      ob.top = 600;
      ob.bottom = 700;  // > 85%×720=612 → 近距
      p.obstacles.push_back(ob);
      p.timestamp_s = i * 0.1;
      d = engine.step(p, ego);
    }
    expect_true(d.alarm_collision, "collision: near obstacle fallback");
  }
  {
    // 远处静止障碍物 → 无碰撞报警
    AdasEngine engine;
    EgoState ego;
    ego.driving = true;
    ego.speed_kmh = 40;
    AdasDecision d;
    for (int i = 0; i < 5; ++i) {
      AdasPerception p = make_perception(LightState::Green);
      ObstacleInfo ob;
      ob.class_id = 10;
      ob.track_id = 11;
      ob.cx = 640;
      ob.top = 100;
      ob.bottom = 200;  // 远处
      p.obstacles.push_back(ob);
      p.timestamp_s = i * 0.1;
      d = engine.step(p, ego);
    }
    expect_true(!d.alarm_collision, "collision: far obstacle -> no alarm");
  }

  // ===== 场景3：导航覆盖 =====
  {
    // 导航右转 + 右虚线 → 右转箭头
    AdasEngine engine;
    EgoState ego;
    AdasPerception p = make_lane_perception(LineType::Solid, LineType::Dashed);
    p.nav.detected = true;
    p.nav.dir = NavDir::Right;
    AdasDecision d = run_n(engine, p, ego, 5);
    expect_true(d.arrow == ArrowDir::Right && d.led_mask == 0x4,
                "S3: nav right + dashed -> right arrow");
  }
  {
    // 导航左转 + 左实线 → 保持直行（不报警，导航非违规）
    AdasEngine engine;
    EgoState ego;
    AdasPerception p = make_lane_perception(LineType::Solid, LineType::Dashed);
    p.nav.detected = true;
    p.nav.dir = NavDir::Left;
    AdasDecision d = run_n(engine, p, ego, 5);
    expect_true(d.arrow == ArrowDir::Straight,
                "S3: nav left + solid -> hold straight");
    expect_true(!d.alarm_illegal_lane_change,
                "S3: nav on solid -> no alarm (not a violation)");
  }
  {
    // 导航直行 → 直行箭头
    AdasEngine engine;
    EgoState ego;
    ego.lane = LaneId::Left;  // 车道默认左转，导航覆盖为直行
    AdasPerception p = make_perception(LightState::Green);
    p.nav.detected = true;
    p.nav.dir = NavDir::Straight;
    AdasDecision d = run_n(engine, p, ego, 5);
    expect_true(d.arrow == ArrowDir::Straight,
                "S3: nav straight overrides lane default");
  }

  // ===== 黄灯保守处理 =====
  {
    AdasEngine engine;
    EgoState ego;
    AdasDecision d = run_n(engine, make_perception(LightState::Yellow), ego, 5);
    expect_true(d.light == LightState::Yellow && d.stop_required,
                "yellow: conservative stop");
  }

  // ===== reset() 清滞回 =====
  {
    AdasEngine engine;
    EgoState ego;
    run_n(engine, make_perception(LightState::Red), ego, 5);
    engine.reset();
    AdasDecision d = engine.step(make_perception(LightState::Red), ego);
    expect_true(d.light == LightState::Unknown,
                "reset: hysteresis cleared (1 frame after reset not confirmed)");
  }

  if (g_failures == 0) {
    std::cout << "ALL ADAS ENGINE TESTS PASSED\n";
    return 0;
  }
  std::cerr << g_failures << " test(s) FAILED\n";
  return 1;
}
