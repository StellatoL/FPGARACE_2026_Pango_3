#include "common/file_video_source.h"
#include <QFileInfo>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <mutex>

namespace adas {
struct FileVideoSource::State {
  std::mutex mutex;
  std::condition_variable changed;
  FramePool pool{8};
  QString path;
  Layout layout = Layout::Mosaic;
  std::function<void(FramePacket)> on_frame;
  std::function<uint64_t()> new_generation;
  uint64_t generation = 0, sequence = 0, control_serial = 0;
  bool stopping = true, paused = false, loop = false, resume_epoch = false;
  double seek_target = -1;
  std::atomic<double> position{0}, duration{0}, fps{0};
};

FileVideoSource::FileVideoSource(QObject *parent) : QThread(parent), state_(new State) {
  qRegisterMetaType<uint64_t>("uint64_t");
}
FileVideoSource::~FileVideoSource() { stop(); }
void FileVideoSource::set_callbacks(std::function<void(FramePacket)> on_frame,
                                    std::function<uint64_t()> new_generation) {
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->on_frame = std::move(on_frame); state_->new_generation = std::move(new_generation);
}
void FileVideoSource::start_file(QString path, uint64_t generation, Layout layout) {
  if (QThread::currentThread() == this) return;
  stop();
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->path = std::move(path); state_->generation = generation; state_->layout = layout;
    state_->stopping = false; state_->paused = false; state_->resume_epoch = false;
    state_->seek_target = -1; ++state_->control_serial;
    state_->position = 0; state_->duration = 0; state_->fps = 0;
  }
  start();
}
void FileVideoSource::stop() {
  { std::lock_guard<std::mutex> lock(state_->mutex); state_->stopping = true; ++state_->control_serial; }
  state_->changed.notify_all();
  if (QThread::currentThread() != this) wait();
}
void FileVideoSource::set_paused(bool paused) {
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->paused == paused || state_->stopping) return;
    if (!paused) state_->resume_epoch = true;
    state_->paused = paused; ++state_->control_serial;
  }
  state_->changed.notify_all();
}
void FileVideoSource::set_loop(bool loop) {
  std::lock_guard<std::mutex> lock(state_->mutex); state_->loop = loop;
}
void FileVideoSource::set_layout(Layout layout) {
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!isRunning()) state_->layout = layout;
}
void FileVideoSource::seek(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0) return;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->stopping || !isRunning()) return;
    const double duration = state_->duration.load();
    state_->seek_target = duration > 0 ? std::min(seconds, std::max(0.0, duration - .001)) : seconds;
    ++state_->control_serial;
  }
  state_->changed.notify_all();
}
double FileVideoSource::position_seconds() const { return state_->position.load(); }
double FileVideoSource::duration_seconds() const { return state_->duration.load(); }
double FileVideoSource::native_fps() const { return state_->fps.load(); }

void FileVideoSource::run() {
  QString path;
  Layout layout;
  uint64_t generation;
  std::function<void(FramePacket)> on_frame;
  std::function<uint64_t()> new_generation;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    path = state_->path; layout = state_->layout; generation = state_->generation;
    on_frame = state_->on_frame; new_generation = state_->new_generation;
  }
  const auto refresh_epoch = [&] {
    const uint64_t next = new_generation ? new_generation() : generation + 1;
    generation = next > generation ? next : generation + 1;
  };
  const auto make_frame = [&](const cv::Mat &input, double pts, bool image) {
    if (input.empty()) return FramePacket{};
    cv::Mat bgr;
    if (input.channels() == 3) bgr = input;
    else if (input.channels() == 4) cv::cvtColor(input, bgr, cv::COLOR_BGRA2BGR);
    else if (input.channels() == 1) cv::cvtColor(input, bgr, cv::COLOR_GRAY2BGR);
    else return FramePacket{};
    const double scale = std::min(1280.0 / bgr.cols, 720.0 / bgr.rows);
    const int width = std::clamp(cvRound(bgr.cols * scale), 1, 1280);
    const int height = std::clamp(cvRound(bgr.rows * scale), 1, 720);
    cv::Mat canvas(720, 1280, CV_8UC3, cv::Scalar::all(0));
    cv::resize(bgr, canvas(cv::Rect((1280-width)/2, (720-height)/2, width, height)),
               cv::Size(width, height), 0, 0, scale < 1 ? cv::INTER_AREA : cv::INTER_LINEAR);
    cv::Mat rgb565;
    cv::cvtColor(canvas, rgb565, cv::COLOR_BGR2BGR565);
    const Stamp stamp{generation, ++state_->sequence, pts, monotonic_seconds(), image};
    return state_->pool.copy(rgb565.data, rgb565.cols, rgb565.rows, static_cast<int>(rgb565.step), stamp, layout);
  };

  try {
    if (!QFileInfo(path).isFile()) { emit error(tr("本地文件不存在：%1").arg(path), generation); return; }
    const std::string filename = QFileInfo(path).absoluteFilePath().toStdString();
    const cv::Mat still = cv::imread(filename, cv::IMREAD_COLOR);
    if (!still.empty()) {
      emit opened(0, 0, true, generation);
      bool resume;
      {
        std::unique_lock<std::mutex> lock(state_->mutex);
        state_->changed.wait(lock, [&] { return state_->stopping || !state_->paused; });
        if (state_->stopping) return;
        resume = state_->resume_epoch; state_->resume_epoch = false;
      }
      if (resume) refresh_epoch();
      FramePacket packet = make_frame(still, 0, true);
      if (!packet) { emit error(tr("无法创建图片帧"), generation); return; }
      {
        std::unique_lock<std::mutex> lock(state_->mutex);
        state_->changed.wait(lock, [&] { return state_->stopping || !state_->paused; });
        if (state_->stopping) return;
        resume = state_->resume_epoch; state_->resume_epoch = false;
      }
      if (resume) { refresh_epoch(); packet.stamp.generation = generation; }
      packet.stamp.receive_s = monotonic_seconds();
      if (on_frame) on_frame(std::move(packet));
      emit ended(generation); return;
    }

    cv::VideoCapture capture(filename);
    if (!capture.isOpened()) { emit error(tr("无法打开本地视频：%1").arg(path), generation); return; }
    double fps = capture.get(cv::CAP_PROP_FPS);
    if (!std::isfinite(fps) || fps <= 0 || fps > 1000) fps = 30;
    const double count = capture.get(cv::CAP_PROP_FRAME_COUNT);
    const double duration = std::isfinite(count) && count > 0 ? count / fps : 0;
    state_->fps = fps; state_->duration = duration;
    emit opened(fps, duration, false, generation);

    double last_decoded = -1, next_sample = -1, frame_index = 0;
    double anchor_pts = 0;
    auto anchor_wall = std::chrono::steady_clock::now();
    bool anchored = false, decoded_any = false;
    for (;;) {
      bool resume;
      double seek_target;
      uint64_t serial;
      {
        std::unique_lock<std::mutex> lock(state_->mutex);
        state_->changed.wait(lock, [&] { return state_->stopping || !state_->paused; });
        if (state_->stopping) return;
        resume = state_->resume_epoch; state_->resume_epoch = false;
        seek_target = state_->seek_target; state_->seek_target = -1;
        serial = state_->control_serial;
      }
      if (seek_target >= 0) {
        if (!capture.set(cv::CAP_PROP_POS_MSEC, seek_target * 1000)) {
          emit error(tr("当前视频不支持定位"), generation); return;
        }
        const double index = capture.get(cv::CAP_PROP_POS_FRAMES);
        frame_index = std::isfinite(index) && index >= 0 ? index : seek_target * fps;
        last_decoded = -1; next_sample = -1; anchored = false;
      }
      if (resume || seek_target >= 0) { refresh_epoch(); anchored = false; }

      cv::Mat decoded;
      if (!capture.read(decoded) || decoded.empty()) {
        bool loop;
        {
          std::lock_guard<std::mutex> lock(state_->mutex);
          if (state_->stopping) return;
          if (state_->control_serial != serial) continue;
          loop = state_->loop;
        }
        if (!decoded_any) { emit error(tr("视频中没有可解码的帧"), generation); return; }
        if (!loop) { emit ended(generation); return; }
        if (!capture.set(cv::CAP_PROP_POS_FRAMES, 0)) { emit error(tr("视频无法循环播放"), generation); return; }
        refresh_epoch(); frame_index = 0; last_decoded = -1; next_sample = -1; anchored = false;
        continue;
      }
      decoded_any = true;
      double pts = capture.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
      if (!std::isfinite(pts) || pts < 0 || pts <= last_decoded) pts = frame_index / fps;
      if (pts <= last_decoded) pts = last_decoded + 1.0 / fps;
      ++frame_index; last_decoded = pts;
      if (next_sample >= 0 && pts + 1e-6 < next_sample) continue;
      if (next_sample < 0) next_sample = pts;
      do { next_sample += 1.0 / 30; } while (next_sample <= pts + 1e-6);
      if (!anchored) { anchor_pts = pts; anchor_wall = std::chrono::steady_clock::now(); anchored = true; }
      const auto target = anchor_wall + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(pts-anchor_pts));
      {
        std::unique_lock<std::mutex> lock(state_->mutex);
        state_->changed.wait_until(lock, target, [&] { return state_->stopping || state_->control_serial != serial; });
        if (state_->stopping) return;
        if (state_->paused || state_->control_serial != serial) continue;
      }
      FramePacket packet = make_frame(decoded, pts, false);
      {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (state_->stopping) return;
        if (state_->paused || state_->control_serial != serial) continue;
      }
      if (packet) {
        state_->position = pts;
        if (on_frame) on_frame(std::move(packet));
      }
    }
  } catch (const std::exception &exception) {
    emit error(QString::fromUtf8(exception.what()), generation);
  }
}
} // namespace adas
