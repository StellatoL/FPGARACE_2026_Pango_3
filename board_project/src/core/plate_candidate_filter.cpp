// 车牌候选结果精炼过滤器 (v2: 适配 78 类扩展字符集)
//
// 对 YOLOv5+LPRNet 推理产生的候选车牌列表进行三维评分的后处理：
//
// 评分维度：
// 1. 文本质量评分（plate_text_score）：
//    - 长度 7-8 位（标准车牌）+0.30，6 或 9 位 +0.15
//    - 首字符为汉字（省份/港澳等标识）+0.20
//    - 第二位为大写字母（城市代码）+0.15
//    - 尾部字符有效比例 +0.20 × ratio
//    - 数字 ≥ 2 个 +0.10，数字 ≥ 2 且字母 ≤ 2 +0.05
//    - 注意：港/澳/学/警/使/领 等特殊汉字在首字符位置同样得分
//           险/品 等非标准尾部字符会降低尾部分数，但不会完全拒绝
//
// 2. 宽高比评分（plate_aspect_score）：
//    目标宽高比 3.2:1（中国车牌标准比例），偏差越大评分越低
//
// 3. 综合评分（refined_confidence）：
//    检测置信度 × 0.20 + 文本质量 × 0.35 + 宽高比 × 0.45
//
// 过滤策略（v2 放宽以适应港澳/特殊车牌）：
// - 无文本且综合评分 < 0.35 → 丢弃（原 0.45，放宽以容纳非标准格式）
// - IoU ≥ 0.30 的重叠候选 → 保留评分更高的
//
// 输出：稳定排序（评分相同时按面积排序）后的精炼候选列表

#include "core/plate_candidate_filter.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace lpr {
namespace {

// 带评分的候选封装
struct ScoredCandidate {
  PlateDetectionResult candidate;
  float rank_score = 0.0f; // 综合评分
};

// UTF-8 字符串解码为 Unicode 码点序列（用于检测中文字符、数字、字母等）
std::vector<char32_t> utf8_to_codepoints(const std::string &text) {
  std::vector<char32_t> codepoints;
  for (std::size_t i = 0; i < text.size();) {
    const unsigned char byte = static_cast<unsigned char>(text[i]);
    if (byte < 0x80) {
      // ASCII 单字节字符
      codepoints.push_back(static_cast<char32_t>(byte));
      ++i;
      continue;
    }

    if ((byte & 0xE0) == 0xC0 && i + 1 < text.size()) {
      // 双字节 UTF-8 序列
      codepoints.push_back(((byte & 0x1F) << 6) |
                           (static_cast<unsigned char>(text[i + 1]) & 0x3F));
      i += 2;
      continue;
    }

    if ((byte & 0xF0) == 0xE0 && i + 2 < text.size()) {
      // 三字节 UTF-8 序列（中文字符在此范围）
      codepoints.push_back(((byte & 0x0F) << 12) |
                           ((static_cast<unsigned char>(text[i + 1]) & 0x3F) << 6) |
                           (static_cast<unsigned char>(text[i + 2]) & 0x3F));
      i += 3;
      continue;
    }

    if ((byte & 0xF8) == 0xF0 && i + 3 < text.size()) {
      // 四字节 UTF-8 序列
      codepoints.push_back(((byte & 0x07) << 18) |
                           ((static_cast<unsigned char>(text[i + 1]) & 0x3F) << 12) |
                           ((static_cast<unsigned char>(text[i + 2]) & 0x3F) << 6) |
                           (static_cast<unsigned char>(text[i + 3]) & 0x3F));
      i += 4;
      continue;
    }

    ++i; // 非法字节序列，跳过
  }
  return codepoints;
}

bool is_ascii_upper_letter(char32_t cp) {
  return cp >= U'A' && cp <= U'Z';
}

bool is_ascii_digit(char32_t cp) {
  return cp >= U'0' && cp <= U'9';
}

bool is_plate_tail_char(char32_t cp) {
  return is_ascii_upper_letter(cp) || is_ascii_digit(cp);
}

float clamp01(float value) {
  return std::max(0.0f, std::min(1.0f, value));
}

// 文本质量评分：基于中国车牌格式规则（省 + 市 + 5-6 位数字/字母组合）
float plate_text_score(const std::string &text) {
  const std::vector<char32_t> codepoints = utf8_to_codepoints(text);
  if (codepoints.empty()) {
    return 0.0f;
  }

  float score = 0.0f;
  const std::size_t length = codepoints.size();

  // 长度评分（标准车牌 7 位：京A12345 或 8 位：新能源车牌）
  if (length == 7 || length == 8) {
    score += 0.30f;
  } else if (length == 6 || length == 9) {
    score += 0.15f;
  }

  // 首位应为汉字（省份代码）
  if (codepoints.front() > 0x7F) {
    score += 0.20f;
  }

  // 第二位应为大写字母（城市代码）
  if (length >= 2 && is_ascii_upper_letter(codepoints[1])) {
    score += 0.15f;
  }

  // 尾部字符（第 3 位起）应全为数字或大写字母
  int tail_valid = 0;
  int tail_digits = 0;
  int tail_letters = 0;
  for (std::size_t i = 2; i < length; ++i) {
    if (!is_plate_tail_char(codepoints[i])) {
      continue;
    }
    ++tail_valid;
    if (is_ascii_digit(codepoints[i])) {
      ++tail_digits;
    } else {
      ++tail_letters;
    }
  }

  if (length > 2) {
    score += 0.20f * (static_cast<float>(tail_valid) /
                      static_cast<float>(length - 2));
  }

  // 数字/字母分布加分
  if (tail_digits >= 2) {
    score += 0.10f;
  }
  if (tail_letters <= 2 && tail_digits >= 2) {
    score += 0.05f;
  }

  return clamp01(score);
}

// 宽高比评分：目标 3.2:1（中国车牌标准比例 440×140mm ≈ 3.14:1）
float plate_aspect_score(const PlateDetectionResult &candidate) {
  const float width =
      static_cast<float>(std::max(0, candidate.right - candidate.left));
  const float height =
      static_cast<float>(std::max(1, candidate.bottom - candidate.top));
  if (width <= 1.0f || height <= 1.0f) {
    return 0.0f;
  }

  const float aspect = width / height;
  const float delta = std::fabs(aspect - 3.2f); // 与理想宽高比的偏差
  return clamp01(1.0f - delta / 2.8f);          // 偏差 2.8 以内线性衰减
}

// 加权综合评分
float refined_confidence(const PlateDetectionResult &candidate) {
  const float detector_score = clamp01(candidate.confidence);
  const float text_score = plate_text_score(candidate.text);
  const float aspect_score = plate_aspect_score(candidate);
  return clamp01(detector_score * 0.20f + text_score * 0.35f +
                 aspect_score * 0.45f);
}

// 计算两个检测结果的 IoU（交并比），用于去重重叠检测
float intersection_over_union(const PlateDetectionResult &lhs,
                              const PlateDetectionResult &rhs) {
  const int inter_left = std::max(lhs.left, rhs.left);
  const int inter_top = std::max(lhs.top, rhs.top);
  const int inter_right = std::min(lhs.right, rhs.right);
  const int inter_bottom = std::min(lhs.bottom, rhs.bottom);
  const int inter_width = std::max(0, inter_right - inter_left);
  const int inter_height = std::max(0, inter_bottom - inter_top);
  const int inter_area = inter_width * inter_height;
  if (inter_area <= 0) {
    return 0.0f;
  }

  const int lhs_area =
      std::max(0, lhs.right - lhs.left) * std::max(0, lhs.bottom - lhs.top);
  const int rhs_area =
      std::max(0, rhs.right - rhs.left) * std::max(0, rhs.bottom - rhs.top);
  const int union_area = lhs_area + rhs_area - inter_area;
  if (union_area <= 0) {
    return 0.0f;
  }

  return static_cast<float>(inter_area) / static_cast<float>(union_area);
}

// 检测框面积（用于 Tie-breaking 排序）
int box_area(const PlateDetectionResult &candidate) {
  return std::max(0, candidate.right - candidate.left) *
         std::max(0, candidate.bottom - candidate.top);
}

} // namespace

void refine_plate_candidates(std::vector<PlateDetectionResult> *candidates) {
  if (candidates == nullptr || candidates->empty()) {
    return;
  }

  // 步骤 1：多维评分
  std::vector<ScoredCandidate> scored_candidates;
  scored_candidates.reserve(candidates->size());
  for (const auto &candidate : *candidates) {
    scored_candidates.push_back({candidate, refined_confidence(candidate)});
  }

  // 步骤 2：稳定排序（评分降序，同评分面积升序）
  std::stable_sort(scored_candidates.begin(), scored_candidates.end(),
                   [](const ScoredCandidate &lhs, const ScoredCandidate &rhs) {
                     if (lhs.rank_score == rhs.rank_score) {
                       return box_area(lhs.candidate) < box_area(rhs.candidate);
                     }
                     return lhs.rank_score > rhs.rank_score;
                   });

  // 步骤 3+4：低质量过滤 + NMS 去重
  std::vector<PlateDetectionResult> filtered;
  filtered.reserve(candidates->size());
  for (const auto &scored_candidate : scored_candidates) {
    const auto &candidate = scored_candidate.candidate;

    // 无文本且低评分 → 丢弃（v2: 阈值 0.35，低于原 0.45，容纳港澳/特殊车牌）
    if (candidate.text.empty() && scored_candidate.rank_score < 0.35f) {
      continue;
    }

    // IoU 去重：与已保留候选比较
    bool overlaps_existing = false;
    for (const auto &kept : filtered) {
      if (intersection_over_union(candidate, kept) >= 0.30f) {
        overlaps_existing = true;
        break;
      }
    }

    if (!overlaps_existing) {
      filtered.push_back(candidate);
    }
  }

  candidates->swap(filtered);
}

} // namespace lpr
