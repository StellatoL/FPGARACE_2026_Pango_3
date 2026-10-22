#ifndef FRAME_PACKET_H
#define FRAME_PACKET_H

#include "core/adas_types.h"
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>

namespace adas {
inline double monotonic_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct FrameStorage {
  explicit FrameStorage(size_t size) : bytes(size) {}
  std::vector<unsigned char> bytes;
};
struct FramePacket {
  std::shared_ptr<const FrameStorage> pixels;
  Stamp stamp;
  int width = 0, height = 0, stride = 0;
  Layout layout = Layout::Mosaic;
  explicit operator bool() const { return bool(pixels); }
};
struct ViewRect { int x, y, w, h; };
inline ViewRect view_rect(Layout layout, ViewId view, int w, int h) {
  if (layout == Layout::Single) return {0, 0, w, h};
  int v = static_cast<int>(view);
  return {(v & 1) * (w / 2), (v / 2) * (h / 2), w / 2, h / 2};
}
class ViewScheduler {
public:
  ViewId next(Layout layout) {
    if (layout == Layout::Single) return ViewId::Front;
    static constexpr int order[] = {0, 0, 1, 0, 2, 0, 3};
    auto v = static_cast<ViewId>(order[next_]);
    next_ = (next_ + 1) % 7;
    return v;
  }
  void reset() { next_ = 0; }
private:
  unsigned next_ = 0;
};

// Pool holds one reference to each preallocated slot; readers keep immutable
// references. The producer never waits for a slot or overwrites retained data.
class FramePool {
public:
  explicit FramePool(size_t slot_count = 8, int max_w = 1280, int max_h = 720)
      : capacity_(static_cast<size_t>(max_w) * max_h * 2) {
    for (size_t i = 0; i < slot_count; ++i)
      slots_.push_back(std::make_shared<FrameStorage>(capacity_));
  }
  FramePacket copy(const unsigned char *data, int w, int h, int stride,
                   const Stamp &stamp, Layout layout) {
    if (!data || w <= 0 || h <= 0 || w > 1280 || h > 720 ||
        stride < w * 2 || static_cast<size_t>(w) * h * 2 > capacity_)
      return {};
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return {};
    for (auto &slot : slots_) {
      if (slot.use_count() != 1) continue;
      for (int y = 0; y < h; ++y)
        std::memcpy(slot->bytes.data() + static_cast<size_t>(y) * w * 2,
                    data + static_cast<size_t>(y) * stride, w * 2);
      return {slot, stamp, w, h, w * 2, layout};
    }
    return {};
  }
private:
  std::mutex mutex_;
  size_t capacity_;
  std::vector<std::shared_ptr<FrameStorage>> slots_;
};

class LatestFrameMailbox {
public:
  void publish(FramePacket packet) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_) ++overwritten_;
    pending_ = std::move(packet);
  }
  FramePacket take() {
    std::lock_guard<std::mutex> lock(mutex_);
    FramePacket packet = std::move(pending_);
    pending_ = {};
    return packet;
  }
  void clear() { (void)take(); }
  uint64_t overwritten() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return overwritten_;
  }
private:
  mutable std::mutex mutex_;
  FramePacket pending_;
  uint64_t overwritten_ = 0;
};
} // namespace adas
#endif
