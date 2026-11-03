#ifndef DETECTION_MODE_H
#define DETECTION_MODE_H

// ADAS inference and direct video display preserve their existing mode IDs.
enum class DetectionMode {
  RawVideo = 2,         // 纯视频模式（不做推理，仅显示）
  Adas = 3,
};

#endif // DETECTION_MODE_H
