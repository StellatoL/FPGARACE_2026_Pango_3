#ifndef LANE_LINE_ANALYZER_H
#define LANE_LINE_ANALYZER_H

// 车道线虚实分析器（场景2 判决依据）
//
// 算法（经典视觉，OpenCV，运行于推理线程，~3-5ms@A55）：
//   ROI(下半幅) → 灰度 → 高斯去噪 → Sobel|dx| 垂直边缘 → 大津二值化
//   → 左/右半幅列投票找峰值列 → 沿峰值列 ±band 带做行程统计
//   → classify_runs() 分类：实线 / 虚线 / 未知
//
// 设计要点：
// - 无帧间状态：每帧独立分析，时间平滑由 AdasEngine 的 3-of-5 滞回负责，
//   便于单元测试与参数标定。
// - classify_runs() 为纯函数（输入行程向量），可脱离图像独立测试。
// - 同时演示赛题评分项"基础图像处理"（去噪/边缘/二值化）。

#include <vector>

#include <opencv2/core/mat.hpp>

#include "core/adas_types.h"

namespace adas {

// 纯函数：根据行程统计分类车道线虚实（可独立单测）
//
// on_runs:   沿 ROI y 方向的各 ON 连续段长度（至少 1 个元素才可能有效）
// gaps:      相邻 ON 段之间的间隙长度（on_runs.size()-1 个，可为空）
// roi_height: ROI 高度（像素）
// type:      输出分类结果
// conf:      输出置信度 [0,1]
//
// 判决规则：
//   Solid:  最长段 ≥ 55% ROI 高
//   Dashed: 段数 ≥ 3 且总 ON ≥ 35% ROI 高 且间隙长度波动 < 0.5×均值
//           （规律间隙是虚线的判别特征，遮挡/噪声产生的随机断裂不算）
//   Unknown: 其余（证据不足，不授权变道——保守）
void classify_runs(const std::vector<int> &on_runs,
                   const std::vector<int> &gaps, int roi_height,
                   LineType &type, float &conf);

// 车道线虚实分析器
class LaneLineAnalyzer {
public:
  // 分析一帧 BGR 图像，输出左右车道线类型与位置
  // bgr: 1280×720（或其他尺寸）BGR 帧
  // out: 输出结果（left/right 各自独立判定）
  void analyze(const cv::Mat &bgr, LaneLineInfo &out);

  // ---- 可调参数（百分比形式，适配不同分辨率；标定用）----
  int roi_top_pct = 50;    // ROI 顶部 = H * roi_top_pct / 100
  int roi_bot_pct = 92;    // ROI 底部 = H * roi_bot_pct / 100
  int band_half = 10;      // 行程统计列带半宽（像素）
  int min_votes_pct = 20;  // 有效峰值最小票数 = roi_h * min_votes_pct / 100
  int edge_margin = 8;     // 半幅边界排除宽度（像素，避开画面边缘噪声）

private:
  // 单半幅分析：在 [x_begin, x_end) 列范围内找峰值列并做行程统计
  // peak_x: 输出峰值列位置；peak_votes: 输出峰值票数（左右线合理性比较用）
  // 返回 true 表示检出有效车道线
  bool analyze_half(const cv::Mat &binary, int x_begin, int x_end,
                    int roi_height, int &peak_x, int &peak_votes,
                    LineType &type, float &conf);

  // 复用缓冲（避免每帧分配）
  cv::Mat gray_, sobel_abs_, binary_;
  std::vector<int> votes_, votes_smooth_;
  std::vector<int> on_runs_, gaps_;
};

}  // namespace adas

#endif  // LANE_LINE_ANALYZER_H
