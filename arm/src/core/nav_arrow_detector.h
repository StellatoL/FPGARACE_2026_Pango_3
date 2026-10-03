#ifndef NAV_ARROW_DETECTOR_H
#define NAV_ARROW_DETECTOR_H

// 导航箭头识别器（场景3：跟随视频内导航 UI 的箭头图标）
//
// 算法（经典视觉，OpenCV，运行于推理线程，ROI 小、~1-2ms）：
//   可配置 ROI（默认画面左上 32%×30%，导航面板常见位置）
//   → HSV 饱和亮色掩码（蓝/绿，导航箭头常用色）
//   → 闭运算 → 最大轮廓（面积 ≥ 0.3% ROI）
//   → classify_blob_direction() 纯函数判向（质量分布峰值法）
//   → 3-of-5 帧确认 + 丢失 15 帧保持（防闪烁/短暂遮挡）
//
// 判向原理（对实心/描边、直/微弯箭头均适用）：
//   箭头头部（三角形底边）是质量最重处。
//   ← 头部在左 → 列质量峰值偏左；→ 峰值偏右；
//   ↑ 头部在上 → 行质量峰值偏上（列峰值居中时才判上下）。
//
// 备份手段：UI 提供手动导航按键（N 键循环），防现场识别失效。

#include <cstdint>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "core/adas_types.h"

namespace adas {

// 纯函数：根据二值掩码中 blob 的质量分布判箭头方向（可独立单测，无 OpenCV 依赖）
//
// mask: w×h 二值掩码（行优先，非零为箭头像素）
// dir:  输出方向（None=无法判断）
// conf:  输出置信度 [0,1]（峰值偏离中心的程度）
void classify_blob_direction(const std::uint8_t *mask, int w, int h,
                             NavDir &dir, float &conf);

// 导航箭头识别器（含帧间确认状态；实例须与帧序列一一对应使用）
class NavArrowDetector {
public:
  NavArrowDetector();

  // 检测一帧 BGR 图像中的导航箭头（含时间确认）
  void detect(const cv::Mat &bgr, NavArrowInfo &out);

  // ---- ROI 配置（百分比，相对整帧；UI 可调以适配不同导航 UI 布局）----
  void set_roi_pct(int x_pct, int y_pct, int w_pct, int h_pct);
  int roi_x_pct() const { return roi_x_pct_; }
  int roi_y_pct() const { return roi_y_pct_; }
  int roi_w_pct() const { return roi_w_pct_; }
  int roi_h_pct() const { return roi_h_pct_; }

  // 手动导航（备份手段）：直接注入方向，绕过视觉识别
  void set_manual_dir(NavDir dir) { manual_dir_ = dir; }
  NavDir manual_dir() const { return manual_dir_; }

  // 丢失保持帧数（超过后 detected=false）
  static constexpr int kHoldFrames = 15;

private:
  int roi_x_pct_ = 0, roi_y_pct_ = 0, roi_w_pct_ = 32, roi_h_pct_ = 30;
  NavDir manual_dir_ = NavDir::None;   // 手动注入（None=自动识别）
  NavDir confirmed_dir_ = NavDir::None; // 当前已确认方向
  int miss_count_ = 0;                  // 连续未确认帧数
  Confirm5 confirm_[3];                 // Left/Straight/Right 各一个确认器

  // 复用缓冲
  cv::Mat roi_hsv_, roi_mask_, blob_mask_;
  std::vector<std::uint8_t> patch_;
};

}  // namespace adas

#endif  // NAV_ARROW_DETECTOR_H
