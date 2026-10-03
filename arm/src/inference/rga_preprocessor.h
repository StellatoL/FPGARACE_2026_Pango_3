#ifndef RGA_PREPROCESSOR_H
#define RGA_PREPROCESSOR_H

#include <memory>
#include <mutex>

#include <opencv2/core/mat.hpp>

namespace vision {

class RgaLibrary;

// RGA hardware preprocessor — offloads color conversion, resize, and crop to
// the Rockchip RGA 2D accelerator.  Falls back to NEON/OpenCV CPU paths when
// librga.so is unavailable or RGA hardware is absent.
//
// Thread safety: a single internal mutex serializes all RGA HW calls.  This is
// fine because each blit is short (< 2 ms) and contention is low.
class RgaPreprocessor {
public:
  RgaPreprocessor();
  ~RgaPreprocessor();

  RgaPreprocessor(const RgaPreprocessor &) = delete;
  RgaPreprocessor &operator=(const RgaPreprocessor &) = delete;

  // Probe and initialise the RGA library.  Returns true on success.
  bool initialize();

  // True when RGA is ready to use (otherwise CPU fallback).
  bool is_available() const { return rga_available_; }

  // --- per-operation wrappers (each transparently uses RGA or CPU) ---

  // RGB565 raw bytes → BGR888 cv::Mat (CV_8UC3).
  void rgb565_to_bgr(cv::Mat &bgr, const unsigned char *frame565,
                     int width, int height, int stride_bytes);

  // Bilinear resize (src → dst), dst is created/reused automatically.
  void resize(const cv::Mat &src, cv::Mat &dst, int dst_w, int dst_h);

  // Colour-space conversion (currently BGR→RGB / RGB→BGR).
  void cvt_color(const cv::Mat &src, cv::Mat &dst, int conversion_code);

  // Crop an ROI from src, resize to dst_w×dst_h, store in dst.
  void crop_resize(const cv::Mat &src, cv::Mat &dst,
                   const cv::Rect &roi, int dst_w, int dst_h);

  // Combined path for detection preprocessing:
  //   RGB565 → RGB888, simultaneously resized to dst_w×dst_h.
  // This is the fastest single-RGA-pass pipeline.
  void rgb565_to_rgb_resized(const unsigned char *frame565,
                             int width, int height, int stride_bytes,
                             cv::Mat &output, int dst_w, int dst_h);

private:
  std::unique_ptr<RgaLibrary> rga_lib_;
  bool rga_available_ = false;
  std::mutex mutex_;

  // Low-level wrapper around c_RkRgaBlit (MMU / virtual-address mode).
  // Returns true on success.
  bool rga_blit(const void *src_va, int src_w, int src_h, int src_stride,
                int src_format, int src_x, int src_y,
                void *dst_va, int dst_w, int dst_h, int dst_stride,
                int dst_format, int dst_x, int dst_y);

  // Runtime self-check: run RGB565->BGR on a synthetic image through both the
  // RGA hardware path and the CPU reference, and confirm the outputs agree
  // within tolerance. Returns true only when RGA produces trustworthy results.
  // This gates hardware use so a broken/quirky RGA driver silently falls back
  // to CPU instead of feeding garbage pixels into the NN pipeline.
  bool self_check_rgb565_to_bgr();
};

} // namespace vision

#endif // RGA_PREPROCESSOR_H
