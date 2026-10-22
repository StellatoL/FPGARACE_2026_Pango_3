#ifndef PLATE_CANDIDATE_FILTER_H
#define PLATE_CANDIDATE_FILTER_H

#include "core/plate_recognizer.h"

namespace lpr {

// 车牌候选结果精炼：对 YOLOv5+LPRNet 推理产生的候选车牌列表进行后处理
//
// 处理步骤：
// 1. 多维评分 — 综合文本质量（长度、省份汉字、尾部数字/字母分布）、
//    宽高比合理性、检测置信度，计算加权综合评分
// 2. 按评分排序 — 高评分候选优先
// 3. 低质量过滤 — 无文本且评分 < 0.45 的候选丢弃
// 4. NMS 去重 — 基于 IoU 阈值 0.30 移除重叠候选
void refine_plate_candidates(std::vector<PlateDetectionResult> *candidates);

} // namespace lpr

#endif // PLATE_CANDIDATE_FILTER_H
