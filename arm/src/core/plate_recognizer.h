#ifndef PLATE_RECOGNIZER_H
#define PLATE_RECOGNIZER_H

#include <string>
#include <vector>

// 单张车牌检测结果：包含位置、置信度、识别文本、颜色和类型
struct PlateDetectionResult {
  int left = 0;               // 检测框左边界（像素）
  int top = 0;                // 检测框上边界（像素）
  int right = 0;              // 检测框右边界（像素）
  int bottom = 0;             // 检测框下边界（像素）
  float confidence = 0.0f;   // 综合置信度
  std::string text;           // 车牌文本（UTF-8 编码的中文车牌号）
  std::string color;          // 车牌颜色：蓝/绿/黄/白/黑（来自模型颜色输出）
  std::string plate_type;     // 车牌类型：燃油车/新能源/大型车/教练车/警车/军车/使馆车/港澳车/货车等
};

// 一帧图像的车牌识别聚合结果
struct PlateFrameResult {
  bool model_ready = false;                       // 模型是否已加载
  std::string error_message;                      // 错误信息
  std::vector<PlateDetectionResult> plates;       // 检测到的所有车牌
};

// 车牌识别器：YOLOv5 车牌检测 + LPRNet 车牌字符识别 两阶段流水线
//
// 推理流程：
// 1. RGB565 → BGR 颜色转换（NEON SIMD 加速）
// 2. 缩放到检测模型输入尺寸 → RKNN YOLOv5 推理 → 后处理（NMS）
// 3. 对每个检测框：裁剪 → 缩放到 94×24 → RKNN LPRNet 推理
// 4. CTC 贪心解码 → 候选过滤（文本质量 + 宽高比 + IoU 去重）
//
// 使用 PIMPL 模式隐藏 NPU 相关实现细节
class PlateRecognizer {
public:
  PlateRecognizer();
  ~PlateRecognizer();

  // 初始化：加载 YOLOv5 检测模型和 LPRNet 识别模型
  // base_dir: 可执行文件所在目录，模型在 models/ 子目录中搜索
  bool initialize(const std::string &base_dir);

  // 对一帧 RGB565 图像执行车牌检测 + 识别
  // frame565: 原始 RGB565 数据指针
  // width/height: 图像宽高
  // stride_bytes: 行步长
  PlateFrameResult infer_rgb565(const unsigned char *frame565, int width,
                                int height, int stride_bytes);

private:
  // 确保两个模型均已加载（initialize 的内部实现）
  bool ensure_models_loaded(const std::string &base_dir);

  // CTC 贪心解码：将 LPRNet 输出的 logits 转换为车牌文本
  // time_first: false=[classes,time_steps] 旧布局, true=[time_steps,classes] 新布局
  std::string decode_plate_text(const float *logits, int class_count,
                                int time_steps, bool time_first) const;

  std::string detector_model_path_;   // YOLOv5 检测模型路径
  std::string recognizer_model_path_; // LPRNet 识别模型路径
  std::string last_error_;            // 最近一次错误信息
  bool initialized_ = false;          // 模型是否已初始化

  class Impl;                         // 前向声明：PIMPL 隐藏 RKNN 细节
  Impl *impl_ = nullptr;
};

#endif // PLATE_RECOGNIZER_H
