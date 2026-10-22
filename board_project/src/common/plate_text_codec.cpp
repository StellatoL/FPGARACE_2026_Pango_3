// 中国车牌字符集定义与 CTC 贪心解码实现 — v2: 78 类扩展字符集
//
// 字符集说明（78 个字符，匹配 plate_recognition_color 模型）：
// - 索引 0：CTC 空白符 "#"（blank_index）
// - 索引 1-31：31 个省级行政区汉字（含"新"）
// - 索引 32-41：特殊字符 学警港澳挂使领民航危（10 个）
// - 索引 42-51：数字 0-9
// - 索引 52-75：大写字母 A-Z（不含 I O）
// - 索引 76-77：险、品（2 个）

#include "common/plate_text_codec.h"

#include <algorithm>
#include <cctype>

namespace lpr {

const std::vector<std::string> &plate_charset() {
  static const std::vector<std::string> kPlateChars = {
      //  0: CTC 空白符
      "#",
      //  1-31: 省级行政区汉字（31 个）
      u8"京", u8"沪", u8"津", u8"渝", u8"冀", u8"晋", u8"蒙", u8"辽",
      u8"吉", u8"黑", u8"苏", u8"浙", u8"皖", u8"闽", u8"赣", u8"鲁",
      u8"豫", u8"鄂", u8"湘", u8"粤", u8"桂", u8"琼", u8"川", u8"贵",
      u8"云", u8"藏", u8"陕", u8"甘", u8"青", u8"宁", u8"新",
      // 32-41: 特殊字符（10 个）
      u8"学", u8"警", u8"港", u8"澳", u8"挂",
      u8"使", u8"领", u8"民", u8"航", u8"危",
      // 42-51: 数字 0-9（10 个）
      "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
      // 52-75: 大写字母 A-Z 不含 I/O（24 个）
      "A", "B", "C", "D", "E", "F", "G", "H",
      "J", "K", "L", "M", "N", "P", "Q", "R",
      "S", "T", "U", "V", "W", "X", "Y", "Z",
      // 76-77: 险、品（2 个）
      u8"险", u8"品",
  };
  return kPlateChars;
}

int blank_index() { return 0; } // "#" 在索引 0

std::string normalize_plate_text(const std::string &plate_text) {
  // 去除首尾的 "#" 空白符残留
  std::string result = plate_text;
  result.erase(std::remove(result.begin(), result.end(), '#'), result.end());
  return result;
}

// CTC 贪心解码：
// 1. 遍历每个时间步，取概率最大的类别索引
// 2. 跳过空白符（blank_index = 0）
// 3. 合并连续重复索引（CTC 的重复 → 合并规则）
// 4. 过滤无效索引（越界值重置 previous 为 blank）
std::string decode_plate_indices(const std::vector<int> &indices) {
  const auto &charset = plate_charset();
  const int blank = blank_index();
  std::string plate_text;
  int previous = blank;

  for (const int index : indices) {
    if (index < 0 || index >= static_cast<int>(charset.size())) {
      previous = blank;
      continue;
    }
    if (index == blank) {
      previous = blank;
      continue;
    }
    if (index == previous) {
      continue;
    }
    plate_text += charset[static_cast<std::size_t>(index)];
    previous = index;
  }

  return normalize_plate_text(plate_text);
}

} // namespace lpr
