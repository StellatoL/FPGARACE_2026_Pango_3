#ifndef RGA_PREPROCESSOR_H
#define RGA_PREPROCESSOR_H

#include <memory>
#include <mutex>

#include <opencv2/core/mat.hpp>

namespace vision {

class RgaLibrary;

// Hardware-only RGB565 crop, resize and RGB conversion for ADAS input.
// Unavailable or invalid RGA output prevents inference preprocessing.
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

  // True when the RGA runtime passed its synthetic pixel self-check.
  bool is_available() const { return rga_available_; }

  // ADAS contract: hardware only. Coordinates refer to the complete owned
  // RGB565 frame; failure never invokes any CPU image conversion or resize.
  bool rgb565_crop_to_rgb_strict(const unsigned char *frame565, int width,
      int height, int stride_bytes, const cv::Rect &crop, cv::Mat &output,
      int dst_w, int dst_h, const cv::Rect &destination, unsigned char padding = 0);

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
  // This gates hardware use; failure disables inference preprocessing.
  // The CPU reference only processes the synthetic self-check image.
  bool self_check_rgb565_to_bgr();
};

} // namespace vision

#endif // RGA_PREPROCESSOR_H
