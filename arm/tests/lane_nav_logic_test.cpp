// 车道线行程分类 + 导航箭头判向 纯函数单元测试（无 Qt/OpenCV 依赖）
//
// 覆盖：
//   classify_runs()：实线/虚线/未知/遮挡断裂（不应误判为虚线）
//   classify_blob_direction()：←/→/↑/对称形状/空掩码
//
// 构建（板端）：
//   g++ -std=c++17 -O2 -I../src tests/lane_nav_logic_test.cpp \
//       ../src/core/nav_arrow_detector.cpp -o /tmp/lane_nav_test \
//       $(pkg-config --cflags --libs opencv4) && /tmp/lane_nav_test
//   （classify_blob_direction 本身无 OpenCV 依赖，但与检测器同编译单元）

#include "core/lane_line_analyzer.h"
#include "core/nav_arrow_detector.h"

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

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

// 在 w×h 掩码上画一个实心左箭头（头在左，杆在右）
std::vector<std::uint8_t> make_left_arrow(int w, int h) {
  std::vector<std::uint8_t> m(w * h, 0);
  // 头：三角形，尖点 (4, h/2)，底边在 x=30，高 40
  for (int x = 4; x <= 30; ++x) {
    const int half = (x - 4) * 20 / 26;  // 线性展开到 ±20
    for (int y = h / 2 - half; y <= h / 2 + half; ++y) {
      if (y >= 0 && y < h) m[y * w + x] = 255;
    }
  }
  // 杆：x∈[30, w-4]，高 12
  for (int x = 30; x < w - 4; ++x)
    for (int y = h / 2 - 6; y <= h / 2 + 6; ++y) {
      if (y >= 0 && y < h) m[y * w + x] = 255;
    }
  return m;
}

// 右箭头（镜像）
std::vector<std::uint8_t> make_right_arrow(int w, int h) {
  std::vector<std::uint8_t> m = make_left_arrow(w, h);
  std::vector<std::uint8_t> r(w * h, 0);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) r[y * w + x] = m[y * w + (w - 1 - x)];
  return r;
}

// 上箭头（头在上）
std::vector<std::uint8_t> make_up_arrow(int w, int h) {
  std::vector<std::uint8_t> m(w * h, 0);
  // 头：三角形，尖点 (w/2, 4)，底边在 y=30，宽 40
  for (int y = 4; y <= 30; ++y) {
    const int half = (y - 4) * 20 / 26;
    for (int x = w / 2 - half; x <= w / 2 + half; ++x) {
      if (x >= 0 && x < w) m[y * w + x] = 255;
    }
  }
  // 杆：y∈[30, h-4]，宽 12
  for (int y = 30; y < h - 4; ++y)
    for (int x = w / 2 - 6; x <= w / 2 + 6; ++x) {
      if (x >= 0 && x < w) m[y * w + x] = 255;
    }
  return m;
}

}  // namespace

int main() {
  using namespace adas;

  // ===== classify_runs：实线 =====
  {
    LineType type;
    float conf;
    // 一段 200px 连续（ROI 高 300）→ Solid
    classify_runs({200}, {}, 300, type, conf);
    expect_true(type == LineType::Solid, "runs: single long run -> Solid");
    expect_true(conf > 0.5f, "runs: solid confidence");
  }
  {
    // 165px（55% 阈值边界以上）→ Solid
    LineType type;
    float conf;
    classify_runs({165}, {}, 300, type, conf);
    expect_true(type == LineType::Solid, "runs: 165/300 -> Solid");
  }

  // ===== classify_runs：虚线 =====
  {
    // 4 段 × 40px，间隙 45px 均匀（ROI 300）→ Dashed
    LineType type;
    float conf;
    classify_runs({40, 40, 40, 40}, {45, 45, 45}, 300, type, conf);
    expect_true(type == LineType::Dashed, "runs: 4 regular runs -> Dashed");
    expect_true(conf > 0.3f, "runs: dashed confidence");
  }
  {
    // 3 段（最少段数）→ Dashed
    LineType type;
    float conf;
    classify_runs({50, 50, 50}, {50, 50}, 300, type, conf);
    expect_true(type == LineType::Dashed, "runs: 3 runs -> Dashed");
  }

  // ===== classify_runs：未知/遮挡 =====
  {
    // 2 段（不足 3 段）且最长段 < 55% → Unknown（遮挡断裂不算虚线）
    LineType type;
    float conf;
    classify_runs({100, 100}, {80}, 300, type, conf);
    expect_true(type == LineType::Unknown,
                "runs: 2 broken segments -> Unknown (occlusion)");
  }
  {
    // 间隙不均匀（45, 120, 30）→ Unknown
    LineType type;
    float conf;
    classify_runs({40, 40, 40, 40}, {45, 120, 30}, 300, type, conf);
    expect_true(type == LineType::Unknown,
                "runs: irregular gaps -> Unknown");
  }
  {
    // 总量不足（4 段 × 20 = 80 < 35%×300=105）→ Unknown
    LineType type;
    float conf;
    classify_runs({20, 20, 20, 20}, {50, 50, 50}, 300, type, conf);
    expect_true(type == LineType::Unknown, "runs: insufficient total -> Unknown");
  }
  {
    // 空输入 → Unknown
    LineType type;
    float conf;
    classify_runs({}, {}, 300, type, conf);
    expect_true(type == LineType::Unknown, "runs: empty -> Unknown");
  }

  // ===== classify_blob_direction：左/右/上箭头 =====
  {
    NavDir dir;
    float conf;
    const auto left = make_left_arrow(100, 60);
    classify_blob_direction(left.data(), 100, 60, dir, conf);
    expect_true(dir == NavDir::Left, "blob: left arrow -> Left");
  }
  {
    NavDir dir;
    float conf;
    const auto right = make_right_arrow(100, 60);
    classify_blob_direction(right.data(), 100, 60, dir, conf);
    expect_true(dir == NavDir::Right, "blob: right arrow -> Right");
  }
  {
    NavDir dir;
    float conf;
    const auto up = make_up_arrow(60, 100);
    classify_blob_direction(up.data(), 60, 100, dir, conf);
    expect_true(dir == NavDir::Straight, "blob: up arrow -> Straight");
  }
  {
    // 对称形状（实心方块）→ None
    NavDir dir;
    float conf;
    std::vector<std::uint8_t> square(64 * 64, 0);
    for (int y = 16; y < 48; ++y)
      for (int x = 16; x < 48; ++x) square[y * 64 + x] = 255;
    classify_blob_direction(square.data(), 64, 64, dir, conf);
    expect_true(dir == NavDir::None, "blob: symmetric square -> None");
  }
  {
    // 空掩码 → None
    NavDir dir;
    float conf;
    std::vector<std::uint8_t> empty(64 * 64, 0);
    classify_blob_direction(empty.data(), 64, 64, dir, conf);
    expect_true(dir == NavDir::None, "blob: empty mask -> None");
  }

  if (g_failures == 0) {
    std::cout << "ALL LANE/NAV LOGIC TESTS PASSED\n";
    return 0;
  }
  std::cerr << g_failures << " test(s) FAILED\n";
  return 1;
}
