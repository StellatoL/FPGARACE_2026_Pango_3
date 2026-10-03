#ifndef VISION_RESULT_H
#define VISION_RESULT_H

#include <string>
#include <vector>

#include "core/adas_types.h"
#include "inference/detection_mode.h"

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
};

// 告警标记信息：用于在图像上绘制圆形告警指示器
struct OverlayMarkerResult {
  bool visible = false;  // 是否显示告警标记
  int center_x = 0;      // 圆心 X 坐标
  int center_y = 0;      // 圆心 Y 坐标
  int radius = 0;        // 圆半径
  int fill_r = 255;      // 填充颜色 R 分量（默认红色）
  int fill_g = 0;        // 填充颜色 G 分量
  int fill_b = 0;        // 填充颜色 B 分量
  int stroke_r = 255;    // 描边颜色 R 分量（默认白色）
  int stroke_g = 255;    // 描边颜色 G 分量
  int stroke_b = 255;    // 描边颜色 B 分量
  std::string text;      // 告警文字（默认 "!"）
};

// 视觉帧推理结果：聚合一帧图像的所有检测/识别结果
struct VisionFrameResult {
  DetectionMode mode = DetectionMode::PlateRecognition; // 当前检测模式
  bool model_ready = false;    // 模型是否已加载并就绪
  std::string error_message;   // 错误信息（模型未就绪或推理异常时）
  std::string status_text;     // 状态栏文本（如告警描述）
  std::vector<OverlayBoxResult> overlays; // 所有检测框叠加信息
  OverlayMarkerResult alert_marker;       // 告警标记
  adas::AdasPerception adas;   // ADAS 感知快照（仅 Adas 模式填充）
};

#endif // VISION_RESULT_H
