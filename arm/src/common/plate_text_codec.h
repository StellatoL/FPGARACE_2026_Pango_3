#ifndef PLATE_TEXT_CODEC_H
#define PLATE_TEXT_CODEC_H

#include <string>
#include <vector>

namespace lpr {

// 获取中国车牌字符集（78 个字符）
// 包含：31 省汉字 + 10 特殊字符 + 10 数字 + 24 大写字母 + 2 特殊 + 空白符
const std::vector<std::string> &plate_charset();

// 车牌文本规范化（当前为透传，预留扩展点）
std::string normalize_plate_text(const std::string &plate_text);

// 获取 CTC 空白符在字符集中的索引（最后一个位置）
int blank_index();

// CTC 贪心解码：将模型输出的类别索引序列解码为车牌文本
// 规则：过滤空白符、合并连续重复字符、过滤无效索引
// indices: 每个时间步的最大概率类别索引
std::string decode_plate_indices(const std::vector<int> &indices);

} // namespace lpr

#endif // PLATE_TEXT_CODEC_H
