// 推理工作线程实现
//
// 架构说明：
// 本线程是 FPGA→ARM 推理管线的核心消费端，实现了三缓冲零拷贝帧池：
// - 主线程通过 submit_frame() 提交帧指针（仅保存指针，不拷贝像素数据）
// - 推理线程从缓冲池取出帧，执行 NPU 推理，结果通过信号回传主线程
// - 当三缓冲全满时，丢弃新帧以保证推理不积压
//
// 支持两种检测模式的热切换：
// - PlateRecognition: YOLOv5 车牌检测 + LPRNet 字符识别
// - TrafficScene: YOLOv5 交通场景检测（斑马线/行人/红绿灯）

#include "inference/plate_infer_thread.h"

#include <QMutexLocker>

#include "core/traffic_scene_detector.h"

#include <sstream>

namespace {

// 根据车牌类型返回对应的检测框 RGB 颜色
// 优先按类型着色，无法判定时回退模型颜色
struct PlateTypeStyle { const char *keyword; int r, g, b; };
const PlateTypeStyle kPlateTypeStyles[] = {
  {u8"教练车", 255, 200, 0},   // 黄
  {u8"警车",   255, 255, 255}, // 白
  {u8"军车",   255, 255, 255}, // 白
  {u8"使馆车", 60,  60,  60},  // 黑
  {u8"港澳车", 60,  60,  60},  // 黑
  {u8"货车",   255, 200, 0},   // 黄
  {u8"新能源", 0,   200, 80},  // 绿
  {u8"标准车牌", 0,  100, 255}, // 蓝
};

// 模型颜色名称 → 框颜色
struct PlateColorStyle { const char *name; int r, g, b; };
const PlateColorStyle kPlateColorStyles[] = {
  {u8"蓝", 0,   100, 255},
  {u8"绿", 0,   200, 80},
  {u8"黄", 255, 200, 0},
  {u8"白", 220, 220, 220},
  {u8"黑", 60,  60,  60},
};

void set_overlay_color(OverlayBoxResult &overlay,
                       const std::string &plate_type,
                       const std::string &color_name) {
  // 优先按类型着色
  for (const auto &style : kPlateTypeStyles) {
    if (plate_type.find(style.keyword) != std::string::npos) {
      overlay.color_r = style.r; overlay.color_g = style.g; overlay.color_b = style.b;
      return;
    }
  }
  // 回退模型颜色
  for (const auto &style : kPlateColorStyles) {
    if (color_name == style.name) {
      overlay.color_r = style.r; overlay.color_g = style.g; overlay.color_b = style.b;
      return;
    }
  }
  // fallback
  overlay.color_r = 0; overlay.color_g = 255; overlay.color_b = 0;
}

// 将 PlateRecognizer 的原始输出转换为统一的 VisionFrameResult 格式
// 用于绘制检测框叠加层（颜色框 + 车牌文本 + 类型标签）
VisionFrameResult make_plate_overlay_result(const PlateFrameResult &plate_result) {
  VisionFrameResult vision_result;
  vision_result.mode = DetectionMode::PlateRecognition;
  vision_result.model_ready = plate_result.model_ready;
  vision_result.error_message = plate_result.error_message;

  for (const auto &plate : plate_result.plates) {
    OverlayBoxResult overlay;
    overlay.left = plate.left;
    overlay.top = plate.top;
    overlay.right = plate.right;
    overlay.bottom = plate.bottom;
    overlay.confidence = plate.confidence;

    set_overlay_color(overlay, plate.plate_type, plate.color);

    std::ostringstream label;
    if (!plate.text.empty()) {
      label << plate.text; // 车牌号
    }
    if (!plate.plate_type.empty()) {
      label << " " << plate.plate_type; // 类型标签
    }
    if (label.tellp() == 0) {
      label << "plate";
    }
    label << " " << plate.confidence;
    overlay.label = label.str();
    vision_result.overlays.push_back(overlay);
  }

  return vision_result;
}

} // namespace

PlateInferThread::PlateInferThread(QObject *parent) : QThread(parent) {
  // 注册自定义类型，使其可在跨线程信号/槽中传递
  qRegisterMetaType<VisionFrameResult>("VisionFrameResult");
}

PlateInferThread::~PlateInferThread() {
  stop();
  wait(); // 等待线程体 run() 退出
}

void PlateInferThread::set_model_root(const QString &model_root) {
  QMutexLocker locker(&mutex_);
  model_root_ = model_root;
}

void PlateInferThread::set_mode(DetectionMode mode) {
  QMutexLocker locker(&mutex_);
  mode_ = mode;
}

void PlateInferThread::submit_frame(quint64 sequence,
                                    const unsigned char *frame565, int width,
                                    int height, int stride_bytes) {
  // 参数合法性检查
  if (frame565 == nullptr || width <= 0 || height <= 0 || stride_bytes <= 0) {
    return;
  }

  QMutexLocker locker(&mutex_);

  // 三缓冲回环写入逻辑：如果下一个槽位正在被推理消费，丢弃当前帧
  const int next = (write_idx_ + 1) % 3;
  if (frame_pool_[next].in_use) {
    // 所有缓冲槽均被占用，丢弃此帧以避免阻塞 FPGA DMA
    dropped_frames_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  // 指针交换：仅保存原始数据指针，零拷贝
  auto &slot = frame_pool_[next];
  slot.data = frame565;
  slot.width = width;
  slot.height = height;
  slot.stride = stride_bytes;
  slot.sequence = sequence;
  slot.mode = mode_;              // 在锁内读取当前模式，确保一致性
  slot.ready = true;
  write_idx_ = next;
  condition_.wakeOne();           // 唤醒推理线程消费
}

void PlateInferThread::stop() {
  QMutexLocker locker(&mutex_);
  stopping_ = true;
  condition_.wakeOne(); // 唤醒等待中的 run() 以便退出
}

void PlateInferThread::run() {
  // 推理引擎实例：在线程内部创建，确保 NPU 资源在推理线程上下文中使用
  PlateRecognizer plate_recognizer;
  TrafficSceneDetector traffic_detector;

  // 延迟初始化标志：首次收到对应模式的帧时才加载模型
  bool plate_initialized = false;
  bool traffic_initialized = false;

  while (true) {
    int idx;
    QString model_root;
    {
      QMutexLocker locker(&mutex_);

      // 条件等待：有新帧提交或请求停止
      while (!stopping_ && !frame_pool_[write_idx_].ready) {
        condition_.wait(&mutex_);
      }
      if (stopping_) {
        break;
      }

      // 锁定当前槽位进行消费
      idx = write_idx_;
      frame_pool_[idx].in_use = true;
      frame_pool_[idx].ready = false;
      model_root = model_root_;
    }

    auto &slot = frame_pool_[idx];
    VisionFrameResult result;

    // 根据模式选择推理引擎
    if (slot.mode == DetectionMode::RawVideo) {
      // 纯视频模式：不做推理，返回空结果
      result.mode = DetectionMode::RawVideo;
    } else if (slot.mode == DetectionMode::PlateRecognition) {
      if (!plate_initialized) {
        plate_initialized = plate_recognizer.initialize(model_root.toStdString());
      }
      result = make_plate_overlay_result(
          plate_recognizer.infer_rgb565(slot.data, slot.width, slot.height,
                                        slot.stride));
    } else {
      if (!traffic_initialized) {
        traffic_initialized =
            traffic_detector.initialize(model_root.toStdString());
      }
      result = traffic_detector.infer_rgb565(slot.data, slot.width, slot.height,
                                             slot.stride);
    }

    // 异步发送结果到主线程（QueuedConnection 确保 UI 线程安全）
    emit result_ready(slot.sequence, result);

    // 释放槽位，允许提交新帧
    QMutexLocker locker(&mutex_);
    frame_pool_[idx].in_use = false;
  }
}
