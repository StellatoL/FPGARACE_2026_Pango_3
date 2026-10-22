// 视觉处理公共工具函数实现
//
// 关键优化：rgb565_to_bgr 使用 ARM NEON SIMD 指令集
// 一次处理 8 个像素（16 字节加载 → 位提取 → 8 位窄化 → 3 通道交错存储）
// 性能：~3-5ms → <1ms（1280×720 单帧，约 4-6x 加速）

#include "common/vision_common.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

#ifdef PANGO_HAVE_RGA
#include "inference/rga_preprocessor.h"
#endif

namespace vision {

#ifdef PANGO_HAVE_RGA
namespace {
// 进程级共享的 RGA 预处理器。首次使用时初始化并执行自检：
// 只有当 librga 加载成功、且 RGA 的 565→BGR 输出与 CPU 结果在容差内一致时，
// is_available() 才返回 true（详见 RgaPreprocessor::initialize）。否则永久回退
// CPU/NEON 路径。C++11 保证函数内静态变量的线程安全初始化；且推理线程唯一，
// 实际为单线程访问。
RgaPreprocessor &shared_rga() {
  static RgaPreprocessor rga;
  static const bool inited = rga.initialize();
  (void)inited;
  return rga;
}
} // namespace
#endif

void rgb565_to_bgr(cv::Mat &bgr, const unsigned char *frame565, int width,
                   int height, int stride_bytes) {
  // create() 仅在尺寸变化时重新分配，同尺寸帧复用同一 buffer
  bgr.create(height, width, CV_8UC3);

#ifdef PANGO_HAVE_RGA
  // 优先走 RGA 2D 硬件（自检通过时）。RGA 内部若 blit 失败也会自行回退 CPU。
  {
    RgaPreprocessor &rga = shared_rga();
    if (rga.is_available()) {
      rga.rgb565_to_bgr(bgr, frame565, width, height, stride_bytes);
      return;
    }
  }
#endif

#ifdef __ARM_NEON
  // NEON SIMD 快速路径：8 像素 / 次迭代
  for (int y = 0; y < height; ++y) {
    const auto *src =
        reinterpret_cast<const uint16_t *>(frame565 + y * stride_bytes);
    auto *dst = bgr.ptr<uint8_t>(y);

    int x = 0;
    for (; x <= width - 8; x += 8) {
      // 加载 8 个 RGB565 像素（128 位）
      uint16x8_t px = vld1q_u16(src + x);

      // R 通道：取高 5 位 [15:11]，右移 11，左移 3 扩展为 8 位 [0-248]
      uint8x8_t r = vmovn_u16(vshrq_n_u16(px, 11));
      r = vshl_n_u8(r, 3);

      // G 通道：取中间 6 位 [10:5]，右移 5，与 0x3F 掩码，左移 2 扩展
      uint8x8_t g =
          vmovn_u16(vandq_u16(vshrq_n_u16(px, 5), vdupq_n_u16(0x3F)));
      g = vshl_n_u8(g, 2);

      // B 通道：取低 5 位 [4:0]，与 0x1F 掩码，左移 3 扩展
      uint8x8_t b =
          vmovn_u16(vandq_u16(px, vdupq_n_u16(0x1F)));
      b = vshl_n_u8(b, 3);

      // 交错存储为 BGR 三通道（vst3 一次写入 24 字节）
      uint8x8x3_t bgr_pixels = {b, g, r};
      vst3_u8(dst + x * 3, bgr_pixels);
    }

    // 尾部像素标量回退（< 8 个像素）
    for (; x < width; ++x) {
      const uint16_t pixel = src[x];
      dst[x * 3 + 0] = static_cast<uint8_t>((pixel & 0x1F) << 3);
      dst[x * 3 + 1] = static_cast<uint8_t>(((pixel >> 5) & 0x3F) << 2);
      dst[x * 3 + 2] = static_cast<uint8_t>(((pixel >> 11) & 0x1F) << 3);
    }
  }
#else
  // 纯标量回退路径（无 NEON 的 ARM 平台或 x86 调试构建）
  for (int y = 0; y < height; ++y) {
    const auto *src_line =
        reinterpret_cast<const uint16_t *>(frame565 + y * stride_bytes);
    auto *dst_line = bgr.ptr<uint8_t>(y);
    for (int x = 0; x < width; ++x) {
      const uint16_t pixel = src_line[x];
      dst_line[x * 3 + 0] = static_cast<uint8_t>((pixel & 0x1F) << 3);
      dst_line[x * 3 + 1] = static_cast<uint8_t>(((pixel >> 5) & 0x3F) << 2);
      dst_line[x * 3 + 2] = static_cast<uint8_t>(((pixel >> 11) & 0x1F) << 3);
    }
  }
#endif
}

std::string find_first_existing_path(
    const std::vector<std::filesystem::path> &candidates) {
  for (const auto &candidate : candidates) {
    if (std::filesystem::exists(candidate)) {
      return candidate.string();
    }
  }
  return {};
}

std::string read_env_path(const char *name) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return {};
  }
  return value;
}

cv::Rect clip_rect_to_frame(const ztu::nn::image_rect_t &box, int width,
                            int height) {
  // 使用 C++17 std::clamp 将边界框裁剪到帧边界内
  const int left = std::clamp(box.left, 0, width - 1);
  const int top = std::clamp(box.top, 0, height - 1);
  const int right = std::clamp(box.right, 0, width);
  const int bottom = std::clamp(box.bottom, 0, height);
  return cv::Rect(left, top, std::max(0, right - left),
                  std::max(0, bottom - top));
}

} // namespace vision
