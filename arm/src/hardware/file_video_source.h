#ifndef FILE_VIDEO_SOURCE_H
#define FILE_VIDEO_SOURCE_H

// 文件回放视频源：从本地视频/图片文件读取帧，写入与 PCIe DMA 相同的
// img_buf 四槽环形缓冲，并发出 frame_ready(slot) 信号。
//
// 用途：
// 1. 赛题现场 U 盘场景视频的直接演示入口（正式链路为 U 盘→外部播放器→
//    HDMI→FPGA→PCIe→ARM；文件直读作为开发与备份手段，文档中注明）。
// 2. 无 PCIe 设备（/dev/pango_pci_driver 不在位）时的开发验证路径。
//
// 设计要点：
// - 与 pango_fun::run() 同构：写 img_buf + slot*IMG_SIZE，slot 循环 0..3，
//   GUI 侧复用同一个 recv_img_dis(int) 槽，显示/推理管线零改动。
// - slot 循环 0..3 且帧率 ≤30fps → 同一槽的重写周期 ≥133ms，
//   大于推理线程最坏占用（~50ms），与 PCIe 现状同构，无额外竞争。
// - 视频按原生帧率节流（上限 30fps）；图片按 5fps 重发以刷新叠加/HUD。
// - letterbox 缩放到 1280×720（保持宽高比，黑边填充），保证检测坐标
//   与显示坐标系一致。

#include <atomic>
#include <QString>
#include <QThread>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

class FileVideoSource : public QThread {
  Q_OBJECT

public:
  // out_buf: 目标缓冲（pango_fun::img_buf），容量须 ≥ 4*IMG_SIZE
  // out_buf_stride: 单帧字节数（1280*720*2）
  explicit FileVideoSource(unsigned char *out_buf, int frame_bytes,
                           QObject *parent = nullptr);
  ~FileVideoSource() override;

  // 打开视频或图片文件（按扩展名区分）。成功返回 true。
  // 已在播放时打开新文件会先停止当前播放。
  bool open(const QString &path);

  // 停止播放并释放资源（线程安全，需配合 wait()）
  void close();

  bool is_valid() const { return valid_; }
  bool is_image_mode() const { return image_mode_; }

  void set_paused(bool paused) { paused_ = paused; }
  bool is_paused() const { return paused_; }
  void set_loop(bool loop) { loop_ = loop; }
  bool is_loop() const { return loop_; }

  // 当前文件路径（UI 显示用）
  const QString &path() const { return path_; }

  // 播放位置信息（UI 显示用）
  double position_seconds() const { return position_s_; }
  double duration_seconds() const { return duration_s_; }

signals:
  // 一帧已写入 out_buf + slot*frame_bytes（与 pango_fun::send_wr_done 同构）
  void frame_ready(int slot);
  // 播放自然结束（非循环模式）
  void playback_finished();
  // 打开/解码错误
  void source_error(const QString &message);

protected:
  void run() override;

private:
  // 将 BGR 帧缩放并写入当前槽（letterbox + BGR→RGB565）
  void write_frame(const cv::Mat &bgr);

  unsigned char *out_buf_ = nullptr;  // 目标缓冲基址（不拥有）
  int frame_bytes_ = 0;              // 单帧字节数
  QString path_;                     // 当前文件路径
  cv::VideoCapture capture_;         // 视频解码器（视频模式）
  cv::Mat image_frame_;              // 图片模式缓存帧
  bool image_mode_ = false;          // true=图片模式（5fps 重发）
  bool valid_ = false;               // 文件是否成功打开
  std::atomic<bool> stopping_{false};
  std::atomic<bool> paused_{false};
  bool loop_ = true;                 // 默认循环播放（演示场景用）
  int slot_ = 0;                      // 当前写入槽位 0..3
  double fps_ = 25.0;                 // 播放帧率
  double duration_s_ = 0.0;           // 视频时长（图片模式为 0）
  double position_s_ = 0.0;           // 当前播放位置
};

#endif  // FILE_VIDEO_SOURCE_H
