// 文件回放视频源实现
//
// 数据流：cv::VideoCapture / cv::imread → BGR 帧 → letterbox 缩放到
// 1280×720 → BGR→RGB565 → 写入 out_buf + slot*frame_bytes（slot 循环 0..3）
// → emit frame_ready(slot) → MainWindow::recv_img_dis（与 PCIe 同槽）。
//
// 节流：按视频原生 fps（上限 30fps）用 QElapsedTimer 对齐帧间隔；
// 图片模式固定 5fps 重发（刷新叠加/HUD）。

#include "hardware/file_video_source.h"

#include <QElapsedTimer>
#include <QFileInfo>
#include <QString>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>

namespace {

// 判断文件是否为图片（按扩展名，OpenCV imread 支持的常见格式）
bool is_image_file(const QString &path) {
  static const char *kImageExts[] = {".png", ".jpg",  ".jpeg", ".bmp",
                                      ".webp", ".tiff", ".tif"};
  const QString suffix = QFileInfo(path).suffix().toLower();
  for (const char *ext : kImageExts) {
    if (suffix == QString(ext).mid(1)) return true;
  }
  return false;
}

// BGR888 → RGB565（小端，与 FPGA/PCIe 链路格式一致：R[15:11] G[10:5] B[4:0]）
// 1280×720 约 2ms（A55 标量），30fps 内可容纳。
void bgr_to_rgb565(const cv::Mat &bgr, unsigned char *dst) {
  const int rows = bgr.rows, cols = bgr.cols;
  for (int y = 0; y < rows; ++y) {
    const unsigned char *src = bgr.ptr<unsigned char>(y);
    unsigned char *out = dst + y * cols * 2;
    for (int x = 0; x < cols; ++x) {
      const unsigned char b = src[x * 3 + 0];
      const unsigned char g = src[x * 3 + 1];
      const unsigned char r = src[x * 3 + 2];
      const unsigned short px =
          static_cast<unsigned short>(((r >> 3) << 11) | ((g >> 2) << 5) |
                                      (b >> 3));
      out[x * 2 + 0] = static_cast<unsigned char>(px & 0xFF);
      out[x * 2 + 1] = static_cast<unsigned char>(px >> 8);
    }
  }
}

}  // namespace

FileVideoSource::FileVideoSource(unsigned char *out_buf, int frame_bytes,
                                 QObject *parent)
    : QThread(parent), out_buf_(out_buf), frame_bytes_(frame_bytes) {}

FileVideoSource::~FileVideoSource() { close(); }

bool FileVideoSource::open(const QString &path) {
  // 若正在播放，先停止并等待线程退出
  close();

  if (out_buf_ == nullptr || frame_bytes_ <= 0) {
    emit source_error(QStringLiteral("invalid output buffer"));
    return false;
  }

  path_ = path;
  image_mode_ = is_image_file(path);

  if (image_mode_) {
    // 图片模式：cv::imread 读入，缓存后按 5fps 重发
    image_frame_ = cv::imread(path.toStdString(), cv::IMREAD_COLOR);
    if (image_frame_.empty()) {
      valid_ = false;
      emit source_error(QStringLiteral("无法读取图片: %1").arg(path));
      return false;
    }
    valid_ = true;
    fps_ = 5.0;
    duration_s_ = 0.0;
    position_s_ = 0.0;
  } else {
    // 视频模式
    if (!capture_.open(path.toStdString())) {
      valid_ = false;
      emit source_error(QStringLiteral("无法打开视频: %1").arg(path));
      return false;
    }
    const double native_fps = capture_.get(cv::CAP_PROP_FPS);
    fps_ = (native_fps > 1.0 && native_fps < 120.0) ? native_fps : 25.0;
    if (fps_ > 30.0) fps_ = 30.0;  // 上限 30fps
    const int frames = static_cast<int>(
        capture_.get(cv::CAP_PROP_FRAME_COUNT));
    duration_s_ = frames > 0 ? frames / fps_ : 0.0;
    position_s_ = 0.0;
    valid_ = true;
  }

  stopping_ = false;
  paused_ = false;
  start();
  return true;
}

void FileVideoSource::close() {
  stopping_ = true;
  if (isRunning()) {
    wait(3000);
  }
  if (capture_.isOpened()) {
    capture_.release();
  }
  image_frame_.release();
  valid_ = false;
}

void FileVideoSource::write_frame(const cv::Mat &bgr) {
  // letterbox：保持宽高比缩放到 1280×720，黑边填充
  // 目标尺寸与 frame_bytes_ 对应（1280×720×2）
  const int dst_w = 1280, dst_h = 720;
  cv::Mat canvas(dst_h, dst_w, CV_8UC3, cv::Scalar(0, 0, 0));

  if (!bgr.empty()) {
    const double scale = std::min(static_cast<double>(dst_w) / bgr.cols,
                                  static_cast<double>(dst_h) / bgr.rows);
    const int new_w = std::max(1, static_cast<int>(bgr.cols * scale + 0.5));
    const int new_h = std::max(1, static_cast<int>(bgr.rows * scale + 0.5));
    const int off_x = (dst_w - new_w) / 2;
    const int off_y = (dst_h - new_h) / 2;
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(new_w, new_h), 0, 0,
               scale < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
    resized.copyTo(canvas(cv::Rect(off_x, off_y, new_w, new_h)));
  }

  // 写入当前槽并通知（先写数据后发信号，信号跨线程排队天然有序）
  bgr_to_rgb565(canvas, out_buf_ + static_cast<size_t>(slot_) * frame_bytes_);
  emit frame_ready(slot_);
  slot_ = (slot_ + 1) & 3;  // 循环 0..3
}

void FileVideoSource::run() {
  QElapsedTimer pace_timer;
  cv::Mat frame;
  const qint64 frame_interval_ms =
      static_cast<qint64>(1000.0 / fps_ + 0.5);

  while (!stopping_) {
    pace_timer.start();

    if (!paused_) {
      if (image_mode_) {
        // 图片模式：重发缓存帧（刷新叠加/HUD）
        write_frame(image_frame_);
      } else {
        // 视频模式：读取下一帧
        if (!capture_.read(frame) || frame.empty()) {
          // 播放到末尾
          if (loop_) {
            capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
            position_s_ = 0.0;
            continue;  // 重新从头读
          } else {
            emit playback_finished();
            break;
          }
        }
        position_s_ = capture_.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
        write_frame(frame);
      }
    }

    // 帧间隔对齐（扣除本帧处理耗时）
    const qint64 elapsed = pace_timer.elapsed();
    const qint64 sleep_ms = frame_interval_ms - elapsed;
    if (sleep_ms > 1) {
      QThread::msleep(static_cast<unsigned long>(sleep_ms));
    } else {
      QThread::msleep(1);  // 至少让出 1ms，避免忙等占满 CPU
    }
  }
}
