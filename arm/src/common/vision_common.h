#ifndef VISION_COMMON_H
#define VISION_COMMON_H

#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "ztu/nn/structures.hpp"

namespace vision {

// RGB565 格式转 BGR（OpenCV Mat 格式），供推理管线使用
// 支持 ARM NEON SIMD 加速（8 像素并行处理），含标量回退路径
void rgb565_to_bgr(cv::Mat &bgr, const unsigned char *frame565, int width,
                   int height, int stride_bytes);

// 从候选路径列表中查找第一个实际存在的文件路径
// 用于模型文件的多级搜索（当前目录 → 上级目录 → models/ 子目录）
std::string find_first_existing_path(
    const std::vector<std::filesystem::path> &candidates);

// 读取环境变量值，变量不存在或为空时返回空字符串
// 用于支持通过环境变量覆盖模型路径
std::string read_env_path(const char *name);

// 将推理结果的归一化坐标裁剪到帧边界内
// box: 推理坐标（可能越界）；width/height: 帧的实际宽高
// 返回: OpenCV 矩形，确保完全在帧内
cv::Rect clip_rect_to_frame(const ztu::nn::image_rect_t &box, int width,
                            int height);

} // namespace vision

#endif // VISION_COMMON_H
