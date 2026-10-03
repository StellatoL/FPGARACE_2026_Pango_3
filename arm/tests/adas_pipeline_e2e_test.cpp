// ADAS 感知→决策 端到端测试（无 GUI，直接消费合成场景视频）
//
// 用生产代码（LaneLineAnalyzer / NavArrowDetector / AdasEngine）逐帧处理
// tools/gen_adas_test_videos.py 生成的合成视频，验证：
//   场景2：左实右虚 5s → 左虚右实 5s 的车道线分类随时间翻转
//   场景3：导航箭头方向序列 ↑(0-4s) → →(4-8s) → ←(8-12s)
//   决策链：实线+强制变道→报警；虚线+转向灯→变道箭头
//
// 构建（板端，需先生成测试视频）：
//   python3 tools/gen_adas_test_videos.py --outdir /tmp/adas_videos
//   g++ -std=c++17 -O2 -Isrc tests/adas_pipeline_e2e_test.cpp \
//       src/core/lane_line_analyzer.cpp src/core/nav_arrow_detector.cpp \
//       src/core/adas_engine.cpp -o /tmp/adas_e2e \
//       $(pkg-config --cflags --libs opencv4) && /tmp/adas_e2e /tmp/adas_videos

#include "core/adas_engine.h"
#include "core/lane_line_analyzer.h"
#include "core/nav_arrow_detector.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <filesystem>
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

// 逐帧统计场景2 的车道线分类（每秒采样一次，返回每秒的 {左类型, 右类型}）
// LineType: 0=Unknown 1=Solid 2=Dashed
bool scan_lane_video(const std::string &path,
                     std::vector<int> &left_types,
                     std::vector<int> &right_types) {
  cv::VideoCapture cap(path);
  if (!cap.isOpened()) return false;

  adas::LaneLineAnalyzer analyzer;
  cv::Mat frame;
  int frame_idx = 0;
  const int fps = 30;
  int last_sample_sec = -1;

  while (cap.read(frame) && !frame.empty()) {
    const int sec = frame_idx / fps;
    if (sec != last_sample_sec) {
      last_sample_sec = sec;
      adas::LaneLineInfo info;
      analyzer.analyze(frame, info);
      left_types.push_back(info.left_valid
                               ? static_cast<int>(info.left_type) : 0);
      right_types.push_back(info.right_valid
                                ? static_cast<int>(info.right_type) : 0);
    }
    ++frame_idx;
  }
  return !left_types.empty();
}

// 逐帧统计场景3 的导航方向（每秒采样，3-of-5 确认由检测器内部完成）
bool scan_nav_video(const std::string &path,
                    std::vector<int> &dir_per_sec) {
  cv::VideoCapture cap(path);
  if (!cap.isOpened()) return false;

  adas::NavArrowDetector detector;
  cv::Mat frame;
  int frame_idx = 0;
  const int fps = 30;
  int last_sample_sec = -1;

  while (cap.read(frame) && !frame.empty()) {
    const int sec = frame_idx / fps;
    adas::NavArrowInfo info;
    detector.detect(frame, info);
    if (sec != last_sample_sec) {
      last_sample_sec = sec;
      dir_per_sec.push_back(info.detected
                                ? static_cast<int>(info.dir) : 0);
    }
    ++frame_idx;
  }
  return !dir_per_sec.empty();
}

}  // namespace

int main(int argc, char **argv) {
  const std::string dir = argc > 1 ? argv[1] : "/tmp/adas_videos";
  const std::string lane_video = dir + "/scenario2_lane_lines.mp4";
  const std::string nav_video = dir + "/scenario3_navigation.mp4";

  if (!std::filesystem::exists(lane_video) ||
      !std::filesystem::exists(nav_video)) {
    std::cerr << "测试视频不存在，请先运行: python3 tools/gen_adas_test_videos.py"
              << std::endl;
    return 2;
  }

  // ===== 场景2：车道线虚实分类（左实右虚 → 左虚右实）=====
  {
    std::vector<int> left, right;
    if (!scan_lane_video(lane_video, left, right)) {
      expect_true(false, "S2: 视频可解码");
      return 1;
    }
    std::cout << "  每秒左线类型: ";
    for (int t : left) std::cout << t;
    std::cout << "  (1=实 2=虚 0=未知)\n  每秒右线类型: ";
    for (int t : right) std::cout << t;
    std::cout << '\n';

    // 前 5 秒：左=Solid(1)，右=Dashed(2)（允许首秒未确认）
    bool phase1_ok = true;
    for (int s = 1; s < 5 && s < static_cast<int>(left.size()); ++s) {
      if (left[s] != 1 || right[s] != 2) phase1_ok = false;
    }
    expect_true(phase1_ok, "S2: 0-5s 左实线 + 右虚线");

    // 后 5 秒：左=Dashed(2)，右=Solid(1)（允许切换秒过渡）
    bool phase2_ok = true;
    for (int s = 6; s < 10 && s < static_cast<int>(left.size()); ++s) {
      if (left[s] != 2 || right[s] != 1) phase2_ok = false;
    }
    expect_true(phase2_ok, "S2: 5-10s 左虚线 + 右实线");
  }

  // ===== 场景3：导航箭头方向序列 =====
  {
    std::vector<int> dirs;
    if (!scan_nav_video(nav_video, dirs)) {
      expect_true(false, "S3: 视频可解码");
      return 1;
    }
    std::cout << "  每秒导航方向: ";
    for (int d : dirs) std::cout << d;
    std::cout << "  (1=左 2=直 3=右 0=无)\n";

    // 0-4s ↑(2)，4-8s →(3)，8-12s ←(1)（允许边界秒过渡）
    bool up_ok = true, right_ok = true, left_ok = true;
    for (int s = 1; s < 4 && s < (int)dirs.size(); ++s)
      if (dirs[s] != 2) up_ok = false;
    for (int s = 5; s < 8 && s < (int)dirs.size(); ++s)
      if (dirs[s] != 3) right_ok = false;
    for (int s = 9; s < 12 && s < (int)dirs.size(); ++s)
      if (dirs[s] != 1) left_ok = false;
    expect_true(up_ok, "S3: 0-4s 导航直行(↑)");
    expect_true(right_ok, "S3: 4-8s 导航右转(→)");
    expect_true(left_ok, "S3: 8-12s 导航左转(←)");
  }

  // ===== 决策链：感知 → 引擎（用场景2 的实线/虚线段验证变道规则）=====
  {
    // 直接构造与视频一致的感知（视频分类已验证，此处验证引擎消费端）
    adas::AdasEngine engine;
    adas::EgoState ego;
    ego.driving = true;
    ego.speed_kmh = 40;

    // 左实线 + 强制左变道 → 报警
    adas::AdasPerception p;
    p.valid = true;
    p.frame_w = 1280;
    p.frame_h = 720;
    p.lane.left_valid = true;
    p.lane.left_type = adas::LineType::Solid;
    p.lane.right_valid = true;
    p.lane.right_type = adas::LineType::Dashed;
    ego.force_lane_change = adas::TurnSignal::Left;
    adas::AdasDecision d;
    for (int i = 0; i < 5; ++i) {
      p.timestamp_s = i * 0.033;
      d = engine.step(p, ego);
    }
    expect_true(d.alarm_illegal_lane_change,
                "E2E: 实线+强制左变道 → 报警");

    // 切换到虚线段（模拟视频后半段）+ 左转向灯 → 变道箭头，无报警
    engine.reset();
    ego.force_lane_change = adas::TurnSignal::None;
    ego.signal = adas::TurnSignal::Left;
    p.lane.left_type = adas::LineType::Dashed;
    p.lane.right_type = adas::LineType::Solid;
    for (int i = 0; i < 5; ++i) {
      p.timestamp_s = i * 0.033;
      d = engine.step(p, ego);
    }
    expect_true(!d.alarm_illegal_lane_change && d.arrow == adas::ArrowDir::Left,
                "E2E: 虚线+左转向灯 → 左变道箭头，无报警");
  }

  if (g_failures == 0) {
    std::cout << "ALL ADAS E2E TESTS PASSED\n";
    return 0;
  }
  std::cerr << g_failures << " test(s) FAILED\n";
  return 1;
}
