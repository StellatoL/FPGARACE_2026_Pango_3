#include "common/file_video_source.h"
#include <QCoreApplication>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <thread>
#include <unistd.h>

namespace {
struct Sink {
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<adas::FramePacket> frames;
  int ends = 0, errors = 0;
  double fps = 0, duration = 0;
  bool image = false;
  void add(adas::FramePacket p) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!frames.empty()) p.pixels.reset(); // Metadata history must not exhaust the bounded pixel pool.
    frames.push_back(std::move(p)); cv.notify_all();
  }
  bool wait(size_t n) {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, std::chrono::seconds(5), [&] { return frames.size() >= n; });
  }
  bool done() {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, std::chrono::seconds(5), [&] { return ends || errors; });
  }
  size_t count() { std::lock_guard<std::mutex> lock(mutex); return frames.size(); }
};
void connect_source(adas::FileVideoSource &source, Sink &sink, std::atomic<uint64_t> &epoch) {
  source.set_callbacks([&](adas::FramePacket p) { sink.add(std::move(p)); }, [&] { return ++epoch; });
  QObject::connect(&source, &adas::FileVideoSource::opened, &source,
    [&](double fps, double duration, bool image) {
      std::lock_guard<std::mutex> lock(sink.mutex); sink.fps = fps; sink.duration = duration; sink.image = image;
    }, Qt::DirectConnection);
  QObject::connect(&source, &adas::FileVideoSource::ended, &source,
    [&] { std::lock_guard<std::mutex> lock(sink.mutex); ++sink.ends; sink.cv.notify_all(); }, Qt::DirectConnection);
  QObject::connect(&source, &adas::FileVideoSource::error, &source,
    [&](QString) { std::lock_guard<std::mutex> lock(sink.mutex); ++sink.errors; sink.cv.notify_all(); }, Qt::DirectConnection);
}
void image_test(const QString &path) {
  adas::FileVideoSource source; Sink sink; std::atomic<uint64_t> epoch{1};
  connect_source(source, sink, epoch);
  source.start_file(path, 1, adas::Layout::Single);
  assert(sink.done()); source.stop();
  assert(sink.frames.size() == 1 && sink.ends == 1 && sink.errors == 0 && sink.image);
  const auto &p = sink.frames[0];
  assert(p && p.width == 1280 && p.height == 720 && p.stride == 2560 && p.stamp.static_image);
  assert(p.layout == adas::Layout::Single && p.stamp.generation == 1 && p.stamp.receive_s > 0);
  // 80x60 input is letterboxed to 960x720; black padding and red centre.
  const auto &bytes = p.pixels->bytes;
  assert(bytes[0] == 0 && bytes[1] == 0);
  const size_t center = (360 * 1280 + 640) * 2;
  assert(bytes[center] == 0 && bytes[center + 1] == 0xf8);
}
void video_test(const QString &path) {
  adas::FileVideoSource source; Sink sink; std::atomic<uint64_t> epoch{2};
  connect_source(source, sink, epoch);
  const auto begin = std::chrono::steady_clock::now();
  source.start_file(path, 2, adas::Layout::Mosaic);
  assert(sink.done()); source.stop();
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  assert(sink.errors == 0 && sink.ends == 1 && !sink.image);
  assert(sink.frames.size() >= 25 && sink.frames.size() <= 31);
  assert(std::abs(sink.fps - 60) < .1 && std::abs(sink.duration - 1) < .1);
  assert(elapsed > .8 && elapsed < 2.5);
  const auto owned = sink.frames.front().pixels;
  const auto first_pixel = owned->bytes[(360 * 1280 + 640) * 2];
  for (size_t i = 1; i < sink.frames.size(); ++i) {
    assert(sink.frames[i].stamp.source_s > sink.frames[i - 1].stamp.source_s);
    assert(sink.frames[i].stamp.sequence > sink.frames[i - 1].stamp.sequence);
    assert(sink.frames[i].stamp.receive_s >= sink.frames[i - 1].stamp.receive_s);
    assert(!sink.frames[i].stamp.static_image);
  }
  assert(sink.frames.back().stamp.source_s > .9);
  assert(owned->bytes[(360 * 1280 + 640) * 2] == first_pixel);
}
void image_pause_test(const QString &path) {
  adas::FileVideoSource source; Sink sink; std::atomic<uint64_t> epoch{30};
  connect_source(source, sink, epoch);
  std::atomic<bool> opened{false};
  QObject::connect(&source, &adas::FileVideoSource::opened, &source,
    [&](double, double, bool) { source.set_paused(true); opened = true; }, Qt::DirectConnection);
  source.start_file(path, 30, adas::Layout::Single);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!opened && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  assert(opened); std::this_thread::sleep_for(std::chrono::milliseconds(50)); assert(sink.count() == 0);
  source.set_paused(false); assert(sink.done()); source.stop();
  assert(sink.frames.size() == 1 && sink.frames[0].stamp.generation == 31);
}
void pause_loop_seek_test(const QString &path) {
  adas::FileVideoSource source; Sink sink; std::atomic<uint64_t> epoch{10};
  connect_source(source, sink, epoch);
  source.set_loop(true); source.start_file(path, 10, adas::Layout::Single);
  assert(sink.wait(3)); source.set_paused(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  const size_t paused = sink.count();
  std::this_thread::sleep_for(std::chrono::milliseconds(120)); assert(sink.count() == paused);
  source.set_paused(false); assert(sink.wait(paused + 2));
  { std::lock_guard<std::mutex> lock(sink.mutex); assert(sink.frames.back().stamp.generation > 10); }
  source.seek(.75);
  const auto previous_epoch = epoch.load();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  bool restarted_pts = false;
  while (!restarted_pts && std::chrono::steady_clock::now() < deadline) {
    { std::lock_guard<std::mutex> lock(sink.mutex);
      for (size_t i = 1; i < sink.frames.size(); ++i)
        if (sink.frames[i].stamp.generation > sink.frames[i-1].stamp.generation &&
            sink.frames[i].stamp.source_s < sink.frames[i-1].stamp.source_s) restarted_pts = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  assert(epoch.load() >= previous_epoch + 2); // Seek and the following loop both start new epochs.
  source.stop(); assert(!source.isRunning());
  assert(restarted_pts);
  source.start_file(path, 40, adas::Layout::Single);
  source.set_paused(true);
  const auto stop_start = std::chrono::steady_clock::now();
  source.stop(); assert(!source.isRunning());
  assert(std::chrono::duration<double>(std::chrono::steady_clock::now() - stop_start).count() < 1);
  source.start_file("/tmp/adas-replay-missing-file.avi", 50, adas::Layout::Single);
  assert(source.wait(3000)); source.stop(); assert(sink.errors == 1);
}
} // namespace

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  const auto root = std::filesystem::temp_directory_path() / ("adas-replay-test-" + std::to_string(::getpid()));
  std::filesystem::create_directory(root);
  const auto image = root / "still.png", video = root / "clip.avi";
  assert(cv::imwrite(image.string(), cv::Mat(60, 80, CV_8UC3, cv::Scalar(0, 0, 255))));
  cv::VideoWriter writer(video.string(), cv::VideoWriter::fourcc('M','J','P','G'), 60, {80,60});
  assert(writer.isOpened());
  for (int i = 0; i < 60; ++i) writer.write(cv::Mat(60,80,CV_8UC3,cv::Scalar(i*3,80,200)));
  writer.release();
  image_test(QString::fromStdString(image.string())); video_test(QString::fromStdString(video.string()));
  image_pause_test(QString::fromStdString(image.string()));
  pause_loop_seek_test(QString::fromStdString(video.string()));
  std::filesystem::remove_all(root);
  std::cout << "File replay ownership, sampling, static image, epochs, pause, seek and stop: PASS\n";
}
