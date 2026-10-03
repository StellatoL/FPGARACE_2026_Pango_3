#ifndef DETECTION_MODE_H
#define DETECTION_MODE_H

// 检测模式枚举：支持四种模式热切换
enum class DetectionMode {
  PlateRecognition = 0, // 车牌识别模式
  TrafficScene = 1,     // 交通场景检测模式
  RawVideo = 2,         // 纯视频模式（不做推理，仅显示）
  Adas = 3,             // ADAS 辅助驾驶模式（交通检测+车道线虚实+导航箭头+决策引擎）
};

#endif // DETECTION_MODE_H
