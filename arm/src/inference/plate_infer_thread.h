#ifndef PLATE_INFER_THREAD_H
#define PLATE_INFER_THREAD_H

#include <array>
#include <atomic>
#include <cstdint>

#include <QMetaType>
#include <QMutex>
#include <QString>
#include <QThread>
#include <QWaitCondition>

#include "inference/detection_mode.h"
#include "core/plate_recognizer.h"
#include "common/vision_result.h"

// 注册 VisionFrameResult 类型，使其可在 Qt 信号/槽跨线程传递
Q_DECLARE_METATYPE(VisionFrameResult)

// 推理工作线程：接收 FPGA→ARM 的 RGB565 帧，调用 NPU 进行检测/识别
//
// 设计要点：
// - 采用三缓冲指针池（FrameSlot[3]），零拷贝提交帧
// - 提交仅保存原始指针（不复制像素数据），持锁时间 < 100ns
// - 若三缓冲全部占用则丢弃当前帧，确保推理管线不阻塞 FPGA DMA
// - 支持运行时热切换车牌识别 ↔ 交通场景检测模式
// - 推理结果通过 result_ready 信号异步回传主线程
class PlateInferThread : public QThread {
  Q_OBJECT

public:
  explicit PlateInferThread(QObject *parent = nullptr);
  ~PlateInferThread() override;

  // 设置模型文件根目录（模型将在其下级 models/ 或上级目录中搜索）
  void set_model_root(const QString &model_root);

  // 运行时切换检测模式（线程安全）
  void set_mode(DetectionMode mode);

  // 设置导航箭头识别 ROI（百分比，线程安全；MainWindow 的 spinbox 驱动）
  void set_nav_roi(int x_pct, int y_pct, int w_pct, int h_pct);

  // 提交一帧原始图像到推理队列
  // sequence: 帧序号（用于结果排序，丢弃过期结果）
  // frame565: FPGA DMA 缓冲区中的原始 RGB565 图像指针（不拷贝！）
  // width/height: 图像宽高
  // stride_bytes: 行步长（通常为 width*2）
  void submit_frame(quint64 sequence, const unsigned char *frame565, int width,
                    int height, int stride_bytes);

  // 请求线程停止（异步，需配合 wait() 等待退出）
  void stop();

  // 获取自上次查询以来因缓冲满而丢弃的帧数量
  uint64_t dropped_frames() const { return dropped_frames_.load(); }

signals:
  // 推理完成信号，携带帧序号和检测/识别结果
  void result_ready(quint64 sequence, VisionFrameResult result);

protected:
  void run() override;

private:
  // 帧槽位：三缓冲池的基本单元
  struct FrameSlot {
    const unsigned char *data = nullptr; // 指向 FPGA DMA 缓冲区的原始指针
    int width = 0;                       // 图像宽度
    int height = 0;                      // 图像高度
    int stride = 0;                      // 行步长
    quint64 sequence = 0;                // 帧序号
    DetectionMode mode = DetectionMode::PlateRecognition; // 当前检测模式
    bool ready = false;                  // 帧数据就绪标志
    bool in_use = false;                 // 帧正在被推理线程消费的标志
  };

  QMutex mutex_;                         // 保护三缓冲池和模式变量的互斥锁
  QWaitCondition condition_;             // 条件变量：唤醒推理线程处理新帧
  QString model_root_;                   // 模型文件根目录
  DetectionMode mode_ = DetectionMode::PlateRecognition; // 当前检测模式
  int nav_roi_[4] = {0, 0, 32, 30};      // 导航箭头 ROI 百分比 {x,y,w,h}

  std::array<FrameSlot, 3> frame_pool_;  // 三缓冲帧池
  int write_idx_ = 0;                    // 当前可写入槽位索引

  std::atomic<uint64_t> dropped_frames_{0}; // 丢弃帧计数（无锁原子变量）
  bool stopping_ = false;                   // 停止标志
};

#endif // PLATE_INFER_THREAD_H
