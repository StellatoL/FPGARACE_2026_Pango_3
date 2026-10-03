// 导航箭头识别器实现
//
// 流水：ROI 裁剪 → HSV → 蓝/绿双区间掩码 → 闭运算 → 最大轮廓填充
// → classify_blob_direction() 判向 → 3-of-5 确认 + 丢失保持。
//
// 手动导航注入：manual_dir_ != None 时直接采用手动方向（演示备份），
// 视觉识别结果仍持续更新（切回自动时无缝衔接）。

#include "core/nav_arrow_detector.h"

#include <algorithm>
#include <cstring>

#include <opencv2/imgproc.hpp>

namespace adas {

void classify_blob_direction(const std::uint8_t *mask, int w, int h,
                             NavDir &dir, float &conf) {
  dir = NavDir::None;
  conf = 0.0f;

  if (mask == nullptr || w <= 0 || h <= 0) {
    return;
  }

  // ---- 非零像素包围盒 ----
  int x0 = w, x1 = -1, y0 = h, y1 = -1;
  long long total = 0;
  for (int y = 0; y < h; ++y) {
    const std::uint8_t *row = mask + static_cast<size_t>(y) * w;
    for (int x = 0; x < w; ++x) {
      if (row[x]) {
        if (x < x0) x0 = x;
        if (x > x1) x1 = x;
        if (y < y0) y0 = y;
        if (y > y1) y1 = y;
        ++total;
      }
    }
  }
  if (x1 < x0 || total < 16) {
    return;  // 无有效 blob
  }

  const int bw = x1 - x0 + 1;
  const int bh = y1 - y0 + 1;

  // ---- 列/行质量分布（仅包围盒内）----
  std::vector<int> col_mass(bw, 0), row_mass(bh, 0);
  for (int y = y0; y <= y1; ++y) {
    const std::uint8_t *row = mask + static_cast<size_t>(y) * w;
    for (int x = x0; x <= x1; ++x) {
      if (row[x]) {
        ++col_mass[x - x0];
        ++row_mass[y - y0];
      }
    }
  }

  // ---- 质量峰值位置（箭头头部 = 三角形底边，质量最重）----
  int x_peak = 0, y_peak = 0;
  for (int x = 1; x < bw; ++x)
    if (col_mass[x] > col_mass[x_peak]) x_peak = x;
  for (int y = 1; y < bh; ++y)
    if (row_mass[y] > row_mass[y_peak]) y_peak = y;

  const float x_norm = static_cast<float>(x_peak) / static_cast<float>(bw);
  const float y_norm = static_cast<float>(y_peak) / static_cast<float>(bh);

  // ---- 峰值显著性：质量分布平坦（对称形状）时峰值位置无方向信息 ----
  const int col_max = col_mass[x_peak];
  const int col_min = *std::min_element(col_mass.begin(), col_mass.end());
  const bool col_flat = (col_max - col_min) <= col_max / 4;

  // ---- 判向：先左右（列峰值偏移），后上下（行峰值偏移）----
  // ← 头部在左 → 列峰值 < 0.45；→ 头部在右 → 列峰值 > 0.55
  if (!col_flat) {
    if (x_norm < 0.45f) {
      dir = NavDir::Left;
      conf = std::min(1.0f, (0.45f - x_norm) / 0.15f);
      return;
    }
    if (x_norm > 0.55f) {
      dir = NavDir::Right;
      conf = std::min(1.0f, (x_norm - 0.55f) / 0.15f);
      return;
    }
  }
  // ↑ 头部在上 → 行峰值 < 0.45（列方向对称/平坦时才判上下）
  const int row_max = row_mass[y_peak];
  const int row_min = *std::min_element(row_mass.begin(), row_mass.end());
  const bool row_flat = (row_max - row_min) <= row_max / 4;
  if (!row_flat && y_norm < 0.45f) {
    dir = NavDir::Straight;
    conf = std::min(1.0f, (0.45f - y_norm) / 0.15f);
    return;
  }
  // ↓ 或对称形状 → 无法判断
  dir = NavDir::None;
  conf = 0.0f;
}

NavArrowDetector::NavArrowDetector() = default;

void NavArrowDetector::set_roi_pct(int x_pct, int y_pct, int w_pct,
                                   int h_pct) {
  roi_x_pct_ = std::clamp(x_pct, 0, 95);
  roi_y_pct_ = std::clamp(y_pct, 0, 95);
  roi_w_pct_ = std::clamp(w_pct, 5, 100 - roi_x_pct_);
  roi_h_pct_ = std::clamp(h_pct, 5, 100 - roi_y_pct_);
}

void NavArrowDetector::detect(const cv::Mat &bgr, NavArrowInfo &out) {
  out.detected = false;
  out.dir = NavDir::None;
  out.conf = 0.0f;
  out.cx = 0;
  out.cy = 0;

  // ---- 手动导航注入（备份手段）：直接采用，视觉结果仍更新 ----
  NavDir raw_dir = NavDir::None;
  float raw_conf = 0.0f;
  int blob_cx = 0, blob_cy = 0;
  bool blob_found = false;

  if (!bgr.empty()) {
    const int W = bgr.cols, H = bgr.rows;
    const int rx = W * roi_x_pct_ / 100;
    const int ry = H * roi_y_pct_ / 100;
    const int rw = std::max(16, W * roi_w_pct_ / 100);
    const int rh = std::max(16, H * roi_h_pct_ / 100);
    if (rx + rw <= W && ry + rh <= H && rw >= 16 && rh >= 16) {
      const cv::Mat roi(bgr, cv::Rect(rx, ry, rw, rh));

      // ---- HSV 饱和亮色掩码（蓝/绿）----
      cv::cvtColor(roi, roi_hsv_, cv::COLOR_BGR2HSV);
      cv::Mat blue, green;
      // 蓝：H 90-130（OpenCV H∈[0,180]），S>90，V>70
      cv::inRange(roi_hsv_, cv::Scalar(90, 90, 70), cv::Scalar(130, 255, 255),
                  blue);
      // 绿：H 35-85
      cv::inRange(roi_hsv_, cv::Scalar(35, 90, 70), cv::Scalar(85, 255, 255),
                  green);
      cv::bitwise_or(blue, green, roi_mask_);

      // ---- 闭运算连接断裂 ----
      static const cv::Mat kernel =
          cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
      cv::morphologyEx(roi_mask_, roi_mask_, cv::MORPH_CLOSE, kernel);

      // ---- 最大轮廓 ----
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(roi_mask_, contours, cv::RETR_EXTERNAL,
                       cv::CHAIN_APPROX_SIMPLE);
      double best_area = 0.0;
      size_t best_idx = 0;
      const double min_area =
          0.003 * static_cast<double>(rw) * static_cast<double>(rh);
      for (size_t i = 0; i < contours.size(); ++i) {
        const double area = cv::contourArea(contours[i]);
        if (area > best_area) {
          best_area = area;
          best_idx = i;
        }
      }

      if (best_area >= min_area) {
        // ---- 轮廓填充为 blob 掩码 ----
        blob_mask_ = cv::Mat::zeros(roi.size(), CV_8UC1);
        cv::drawContours(blob_mask_, contours,
                         static_cast<int>(best_idx), cv::Scalar(255), -1);

        // ---- 纯函数判向 ----
        classify_blob_direction(blob_mask_.data, blob_mask_.cols,
                                blob_mask_.rows, raw_dir, raw_conf);
        blob_found = (raw_dir != NavDir::None);

        // blob 中心（叠加提示框用）
        const cv::Rect br = cv::boundingRect(contours[best_idx]);
        blob_cx = rx + br.x + br.width / 2;
        blob_cy = ry + br.y + br.height / 2;
      }
    }
  }

  // ---- 3-of-5 帧确认（Left/Straight/Right 各一个确认器）----
  const NavDir kDirs[3] = {NavDir::Left, NavDir::Straight, NavDir::Right};
  bool any_confirmed = false;
  NavDir latest_confirmed = NavDir::None;
  float latest_conf = 0.0f;
  for (int i = 0; i < 3; ++i) {
    const bool confirmed = confirm_[i].update(blob_found && raw_dir == kDirs[i]);
    if (confirmed) {
      any_confirmed = true;
      latest_confirmed = kDirs[i];
      latest_conf = raw_conf;
    }
  }

  if (any_confirmed) {
    confirmed_dir_ = latest_confirmed;
    miss_count_ = 0;
  } else if (confirmed_dir_ != NavDir::None) {
    ++miss_count_;
    if (miss_count_ > kHoldFrames) {
      confirmed_dir_ = NavDir::None;
    }
  }

  // ---- 输出（手动注入优先）----
  if (manual_dir_ != NavDir::None) {
    out.detected = true;
    out.dir = manual_dir_;
    out.conf = 1.0f;
  } else if (confirmed_dir_ != NavDir::None) {
    out.detected = true;
    out.dir = confirmed_dir_;
    out.conf = latest_conf;
    out.cx = blob_cx;
    out.cy = blob_cy;
  }
}

}  // namespace adas
