#ifndef VISION_RESULT_H
#define VISION_RESULT_H

#include <string>
#include <vector>

#include "inference/detection_mode.h"
#include "core/adas_types.h"

// 检测框叠加层信息：用于在图像上绘制检测框及标签
struct OverlayBoxResult {
  int left = 0;          // 检测框左边界坐标（像素）
  int top = 0;           // 检测框上边界坐标（像素）
  int right = 0;         // 检测框右边界坐标（像素）
  int bottom = 0;        // 检测框下边界坐标（像素）
  float confidence = 0.0f; // 检测置信度 [0.0, 1.0]
  int color_r = 0;       // 检测框颜色 R 分量
  int color_g = 255;     // 检测框颜色 G 分量（默认绿色）
  int color_b = 0;       // 检测框颜色 B 分量
  std::string label;     // 叠加显示的文本标签
  int class_id = -1;     // ADAS component routing.
  int64_t track_id = -1;
  bool predicted = false;
  double last_observed_s = 0;
  unsigned track_hits = 0;
};

// 视觉帧推理结果：聚合一帧图像的所有检测/识别结果
struct VisionFrameResult {
  DetectionMode mode = DetectionMode::Adas; // 当前检测模式
  bool model_ready = false;    // 模型是否已加载并就绪
  std::string error_message;   // 错误信息（模型未就绪或推理异常时）
  std::vector<OverlayBoxResult> overlays; // 所有检测框叠加信息
  adas::Stamp stamp;
  adas::ViewId view = adas::ViewId::Front;
  adas::ObjectObservation observation;
};

#endif // VISION_RESULT_H
