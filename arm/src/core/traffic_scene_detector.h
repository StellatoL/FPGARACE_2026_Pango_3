#ifndef TRAFFIC_SCENE_DETECTOR_H
#define TRAFFIC_SCENE_DETECTOR_H

#include <string>

#include <opencv2/core/mat.hpp>

#include "common/vision_result.h"

// 交通场景检测器：使用 YOLOv5 检测斑马线、行人、红绿灯等交通元素
//
// 支持的检测类别（6 类）：
// 0: 斑马线 (zebra)     1: 行人 (person)
// 2: 红灯 (red)         3: 绿灯 (green)
// 4: 黑灯 (black)       5: 黄灯 (yellow)
//
// 使用与车牌检测器相同的统一 YOLOv5 后处理器，仅锚点参数不同
// 采用 PIMPL 模式隐藏 NPU 实现细节
class TrafficSceneDetector {
public:
  TrafficSceneDetector();
  ~TrafficSceneDetector();

  // 初始化：加载交通场景 YOLOv5 RKNN 模型
  bool initialize(const std::string &base_dir);

  // 对一帧 RGB565 图像执行交通场景检测
  // 返回包含检测框叠加信息和告警标记的 VisionFrameResult
  VisionFrameResult infer_rgb565(const unsigned char *frame565, int width,
                                 int height, int stride_bytes);

  // 组装最近一帧的 ADAS 感知快照（须在 infer_rgb565 之后调用）
  // 内容：信号灯状态（最高置信度，红>黄>绿优先）、斑马线/行人标志、
  //       SORT 跟踪障碍物（行人/车辆，含轨迹 ID 与速度）
  adas::AdasPerception build_adas_perception() const;

  // 最近一帧的 BGR 图像（ADAS 感知扩展用：车道线/导航箭头分析输入）
  // 生命周期：下一次 infer_rgb565 调用前有效
  const cv::Mat &last_bgr() const;

private:
  bool ensure_model_loaded(const std::string &base_dir);

  std::string detector_model_path_; // 交通场景检测模型路径
  std::string last_error_;          // 最近一次错误信息
  bool initialized_ = false;        // 模型是否已初始化

  class Impl;                       // PIMPL：隐藏 RKNN 推理引擎细节
  Impl *impl_ = nullptr;
};

#endif // TRAFFIC_SCENE_DETECTOR_H
