// 车道线虚实分析器实现
//
// 处理流水（全部在 ROI 内进行，1280×720 下 ROI 约 1280×300）：
//   1. BGR→灰度 → 高斯 5×5 去噪
//   2. Sobel X → |dx|（垂直边缘响应，车道线近似垂直）
//   3. 大津自适应二值化（白天/夜间亮度差异鲁棒）
//   4. 列投票：每列 ON 像素计数 → 3-tap 平滑 → 左/右半幅各取峰值
//   5. 行程统计：沿峰值列 ±band_half 带逐行判 ON，统计连续段/间隙
//   6. classify_runs() 分类虚实
//
// 性能：灰度+模糊+Sobel+二值 ≈2-3ms，投票+行程 ≈1ms（A55 单核），
// 在推理线程内与 NPU 推理串行执行，30fps 预算内可容纳。

#include "core/lane_line_analyzer.h"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace adas {

void classify_runs(const std::vector<int> &on_runs,
                   const std::vector<int> &gaps, int roi_height,
                   LineType &type, float &conf) {
  type = LineType::Unknown;
  conf = 0.0f;

  if (on_runs.empty() || roi_height <= 0) {
    return;
  }

  // 最长段与总 ON
  int longest = 0;
  int total_on = 0;
  for (int run : on_runs) {
    longest = std::max(longest, run);
    total_on += run;
  }

  // ---- 实线：存在一段足够长的连续行程 ----
  if (longest >= roi_height * 55 / 100) {
    type = LineType::Solid;
    conf = std::min(1.0f, static_cast<float>(longest) /
                              static_cast<float>(roi_height));
    return;
  }

  // ---- 虚线：多段 + 总量足够 + 间隙规律 ----
  if (on_runs.size() >= 3 && total_on >= roi_height * 35 / 100 &&
      gaps.size() >= 2) {
    // 间隙规律性：标准差 < 0.5×均值（虚线间距均匀；遮挡断裂则不均匀）
    double gap_sum = 0.0;
    for (int g : gaps) gap_sum += g;
    const double gap_mean = gap_sum / gaps.size();
    double gap_var = 0.0;
    for (int g : gaps) gap_var += (g - gap_mean) * (g - gap_mean);
    const double gap_std = std::sqrt(gap_var / gaps.size());

    if (gap_mean > 0.0 && gap_std < 0.5 * gap_mean) {
      type = LineType::Dashed;
      // 置信度：总量占比 × 规律性
      const float amount = static_cast<float>(total_on) /
                           static_cast<float>(roi_height);
      const float regular = gap_mean > 0.0
                                ? static_cast<float>(1.0 - gap_std / gap_mean)
                                : 0.0f;
      conf = std::min(1.0f, amount * (0.5f + 0.5f * regular));
      return;
    }
  }

  // 证据不足 → Unknown（引擎侧不授权变道，保守）
  type = LineType::Unknown;
  conf = 0.0f;
}

bool LaneLineAnalyzer::analyze_half(const cv::Mat &binary, int x_begin,
                                     int x_end, int roi_height, int &peak_x,
                                     int &peak_votes, LineType &type,
                                     float &conf) {
  type = LineType::Unknown;
  conf = 0.0f;
  peak_x = -1;
  peak_votes = 0;

  const int width = binary.cols;
  x_begin = std::max(0, x_begin);
  x_end = std::min(width, x_end);
  if (x_end - x_begin < 2 || roi_height <= 0) {
    return false;
  }

  // ---- 列投票 ----
  votes_.assign(width, 0);
  for (int y = 0; y < binary.rows; ++y) {
    const unsigned char *row = binary.ptr<unsigned char>(y);
    for (int x = x_begin; x < x_end; ++x) {
      if (row[x]) ++votes_[x];
    }
  }

  // ---- 3-tap 平滑 [1,2,1]/4 ----
  votes_smooth_.assign(width, 0);
  for (int x = x_begin; x < x_end; ++x) {
    const int left = votes_[std::max(x_begin, x - 1)];
    const int right = votes_[std::min(x_end - 1, x + 1)];
    votes_smooth_[x] = (left + 2 * votes_[x] + right) / 4;
  }

  // ---- 峰值列（排除半幅边界 edge_margin 像素）----
  int best_x = -1, best_votes = 0;
  const int search_begin = x_begin + edge_margin;
  const int search_end = x_end - edge_margin;
  if (search_end - search_begin < 2) {
    return false;
  }
  for (int x = search_begin; x < search_end; ++x) {
    if (votes_smooth_[x] > best_votes) {
      best_votes = votes_smooth_[x];
      best_x = x;
    }
  }

  // 有效阈值：峰值票数不足则视为无车道线
  const int min_votes = std::max(10, roi_height * min_votes_pct / 100);
  if (best_x < 0 || best_votes < min_votes) {
    return false;
  }

  // ---- 行程统计：沿峰值列 ±band_half 带逐行判 ON，同步记录段与间隙 ----
  on_runs_.clear();
  gaps_.clear();
  const int band_left = std::max(0, best_x - band_half);
  const int band_right = std::min(width, best_x + band_half + 1);
  int cur_run = 0;
  int cur_gap = 0;
  bool seen_first_run = false;
  for (int y = 0; y < binary.rows; ++y) {
    const unsigned char *row = binary.ptr<unsigned char>(y);
    bool on = false;
    for (int x = band_left; x < band_right; ++x) {
      if (row[x]) {
        on = true;
        break;
      }
    }
    if (on) {
      ++cur_run;
      if (seen_first_run && cur_run == 1 && cur_gap > 0) {
        gaps_.push_back(cur_gap);  // 上一段结束到本段开始的间隙
      }
      cur_gap = 0;
    } else {
      if (cur_run > 0) {
        on_runs_.push_back(cur_run);
        cur_run = 0;
        seen_first_run = true;
      }
      if (seen_first_run) ++cur_gap;
    }
  }
  if (cur_run > 0) {
    on_runs_.push_back(cur_run);
  }

  classify_runs(on_runs_, gaps_, roi_height, type, conf);
  peak_x = best_x;
  peak_votes = best_votes;

  // 检出有效性：分类为 Unknown 但峰值明显 → 也算"有线但类型未知"
  return type != LineType::Unknown || best_votes >= min_votes * 2;
}

void LaneLineAnalyzer::analyze(const cv::Mat &bgr, LaneLineInfo &out) {
  out = LaneLineInfo();  // 重置为默认（全部无效）

  if (bgr.empty() || bgr.cols < 64 || bgr.rows < 64) {
    return;
  }

  const int width = bgr.cols;
  const int height = bgr.rows;

  // ---- ROI ----
  const int roi_y0 = height * roi_top_pct / 100;
  const int roi_y1 = height * roi_bot_pct / 100;
  const int roi_h = roi_y1 - roi_y0;
  if (roi_h < 32) {
    return;
  }
  const cv::Mat roi(bgr, cv::Rect(0, roi_y0, width, roi_h));

  // ---- 灰度 → 去噪 → Sobel → 二值化 ----
  cv::cvtColor(roi, gray_, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray_, gray_, cv::Size(5, 5), 0);
  cv::Sobel(gray_, sobel_abs_, CV_16S, 1, 0, 3);
  cv::convertScaleAbs(sobel_abs_, sobel_abs_);
  cv::threshold(sobel_abs_, binary_, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU);

  // ---- 左右半幅独立分析 ----
  const int half = width / 2;
  LineType ltype, rtype;
  float lconf, rconf;
  int lpeak, rpeak, l_votes, r_votes;
  const bool left_ok = analyze_half(binary_, 0, half, roi_h, lpeak, l_votes,
                                    ltype, lconf);
  const bool right_ok = analyze_half(binary_, half, width, roi_h, rpeak,
                                      r_votes, rtype, rconf);

  // ---- 合理性检查：左右线过近（<12% 宽度）视为同一条线，保留票数高者 ----
  if (left_ok && right_ok &&
      std::abs(lpeak - rpeak) < width * 12 / 100) {
    if (l_votes >= r_votes) {
      out.left_valid = true;
      out.left_type = ltype;
      out.left_conf = lconf;
      out.left_x = lpeak;
    } else {
      out.right_valid = true;
      out.right_type = rtype;
      out.right_conf = rconf;
      out.right_x = rpeak;
    }
  } else {
    if (left_ok) {
      out.left_valid = true;
      out.left_type = ltype;
      out.left_conf = lconf;
      out.left_x = lpeak;
    }
    if (right_ok) {
      out.right_valid = true;
      out.right_type = rtype;
      out.right_conf = rconf;
      out.right_x = rpeak;
    }
  }

  // ---- 车道中心偏移（偏离预警用）----
  if (out.left_valid && out.right_valid) {
    out.lane_center_offset = (out.left_x + out.right_x) / 2 - width / 2;
  }
}

}  // namespace adas
