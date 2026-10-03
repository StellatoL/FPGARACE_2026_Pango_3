// RGA hardware preprocessor implementation.
//
// Uses dlopen/dlsym to load librga.so at runtime so the binary still works on
// boards without the RGA userspace library.  Falls back to the existing NEON /
// OpenCV CPU paths transparently.

#include "inference/rga_preprocessor.h"

#include <dlfcn.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

// System RGA headers (installed by librga-dev) — used only for type
// definitions; the actual function symbols are loaded via dlsym.
#include <rga/RgaApi.h>
#include <rga/drmrga.h>
#include <rga/rga.h>

// The RgaApi.h header defines convenience macros RgaInit/RgaBlit/RgaDeInit
// that conflict with our function-pointer member names.  Undefine them now
// so we can use the raw c_RkRga* symbols directly.
#undef RgaInit
#undef RgaBlit
#undef RgaDeInit

#include <opencv2/imgproc.hpp>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

namespace vision {

// =========================================================================
// RgaLibrary — thin RAII wrapper around dlopen'd librga.so
// =========================================================================
class RgaLibrary {
public:
  RgaLibrary() = default;

  ~RgaLibrary() {
    if (handle_ != nullptr) {
      if (RgaDeInit != nullptr)
        RgaDeInit();
      dlclose(handle_);
    }
  }

  RgaLibrary(const RgaLibrary &) = delete;
  RgaLibrary &operator=(const RgaLibrary &) = delete;

  bool load() {
    if (handle_ != nullptr)
      return true; // already loaded

    // Try the common soname; some BSPs ship it as librga2.so
    handle_ = dlopen("librga.so", RTLD_LAZY);
    if (handle_ == nullptr) {
      handle_ = dlopen("librga.so.2", RTLD_LAZY);
    }
    if (handle_ == nullptr)
      return false;

    RgaInit = reinterpret_cast<decltype(RgaInit)>(dlsym(handle_, "c_RkRgaInit"));
    RgaBlit = reinterpret_cast<decltype(RgaBlit)>(dlsym(handle_, "c_RkRgaBlit"));
    RgaDeInit =
        reinterpret_cast<decltype(RgaDeInit)>(dlsym(handle_, "c_RkRgaDeInit"));

    if (RgaInit == nullptr || RgaBlit == nullptr || RgaDeInit == nullptr) {
      dlclose(handle_);
      handle_ = nullptr;
      return false;
    }

    if (RgaInit() != 0) {
      dlclose(handle_);
      handle_ = nullptr;
      return false;
    }

    return true;
  }

  bool loaded() const { return handle_ != nullptr; }

  // ===== function pointers =====
  int (*RgaInit)(void) = nullptr;
  int (*RgaBlit)(rga_info_t *, rga_info_t *, rga_info_t *) = nullptr;
  void (*RgaDeInit)(void) = nullptr;

private:
  void *handle_ = nullptr;
};

namespace {

// =========================================================================
// CPU fallback implementations (identical to the current NEON / OpenCV paths)
// =========================================================================

void cpu_rgb565_to_bgr(cv::Mat &bgr, const unsigned char *frame565, int width,
                       int height, int stride_bytes) {
  bgr.create(height, width, CV_8UC3);

#ifdef __ARM_NEON
  for (int y = 0; y < height; ++y) {
    const auto *src =
        reinterpret_cast<const uint16_t *>(frame565 + y * stride_bytes);
    auto *dst = bgr.ptr<uint8_t>(y);

    int x = 0;
    for (; x <= width - 8; x += 8) {
      uint16x8_t px = vld1q_u16(src + x);

      uint8x8_t r = vmovn_u16(vshrq_n_u16(px, 11));
      r = vshl_n_u8(r, 3);

      uint8x8_t g =
          vmovn_u16(vandq_u16(vshrq_n_u16(px, 5), vdupq_n_u16(0x3F)));
      g = vshl_n_u8(g, 2);

      uint8x8_t b = vmovn_u16(vandq_u16(px, vdupq_n_u16(0x1F)));
      b = vshl_n_u8(b, 3);

      uint8x8x3_t bgr_pixels = {b, g, r};
      vst3_u8(dst + x * 3, bgr_pixels);
    }

    for (; x < width; ++x) {
      const uint16_t pixel = src[x];
      dst[x * 3 + 0] = static_cast<uint8_t>((pixel & 0x1F) << 3);
      dst[x * 3 + 1] = static_cast<uint8_t>(((pixel >> 5) & 0x3F) << 2);
      dst[x * 3 + 2] = static_cast<uint8_t>(((pixel >> 11) & 0x1F) << 3);
    }
  }
#else
  for (int y = 0; y < height; ++y) {
    const auto *src_line =
        reinterpret_cast<const uint16_t *>(frame565 + y * stride_bytes);
    auto *dst_line = bgr.ptr<uint8_t>(y);
    for (int x = 0; x < width; ++x) {
      const uint16_t pixel = src_line[x];
      dst_line[x * 3 + 0] = static_cast<uint8_t>((pixel & 0x1F) << 3);
      dst_line[x * 3 + 1] = static_cast<uint8_t>(((pixel >> 5) & 0x3F) << 2);
      dst_line[x * 3 + 2] = static_cast<uint8_t>(((pixel >> 11) & 0x1F) << 3);
    }
  }
#endif
}

void cpu_resize(const cv::Mat &src, cv::Mat &dst, int dst_w, int dst_h) {
  cv::resize(src, dst, cv::Size(dst_w, dst_h), 0, 0, cv::INTER_LINEAR);
}

void cpu_cvt_color(const cv::Mat &src, cv::Mat &dst, int code) {
  cv::cvtColor(src, dst, code);
}

void cpu_crop_resize(const cv::Mat &src, cv::Mat &dst, const cv::Rect &roi,
                     int dst_w, int dst_h) {
  cv::Mat roi_view = src(roi);
  cv::resize(roi_view, dst, cv::Size(dst_w, dst_h), 0, 0, cv::INTER_LINEAR);
}

} // namespace

// =========================================================================
// RgaPreprocessor
// =========================================================================

RgaPreprocessor::RgaPreprocessor() : rga_lib_(new RgaLibrary()) {}

RgaPreprocessor::~RgaPreprocessor() = default;

bool RgaPreprocessor::initialize() {
  if (rga_available_)
    return true;

  // Skip if the environment explicitly disables RGA.
  const char *env = std::getenv("PANGO_DISABLE_RGA");
  if (env != nullptr && env[0] != '\0' && env[0] != '0')
    return false;

  if (!rga_lib_->load())
    return false;

  // The library loaded, but that does not prove the RGA hardware/driver on
  // this board actually produces correct output (some BSPs run in a degraded
  // "compatibility mode"). Validate against the CPU reference before trusting
  // it. rga_available_ must be true during the check so rga_blit() takes the
  // hardware path; we clear it again if the check fails.
  rga_available_ = true;
  if (!self_check_rgb565_to_bgr()) {
    rga_available_ = false;
    return false;
  }

  return true;
}

namespace {
// RGA wstride/hstride must be in pixels, but callers pass byte strides.
// Convert byte stride to pixel stride based on the format's bytes-per-pixel.
int to_pixel_stride(int byte_stride, int format) {
  int bpp;
  switch (format) {
  case 0x400:  // RK_FORMAT_RGB_565
  case 0x500:  // RK_FORMAT_RGBA_5551
  case 0x600:  // RK_FORMAT_RGBA_4444
    bpp = 2; break;
  case 0x200:  // RK_FORMAT_RGB_888
  case 0x300:  // RK_FORMAT_BGRA_8888
  case 0x700:  // RK_FORMAT_BGR_888
    bpp = 3; break;
  case 0x800:  // RK_FORMAT_RGBA_8888
  case 0x900:  // RK_FORMAT_BGRA_8888 (alternative)
    bpp = 4; break;
  case 0x1300: // RK_FORMAT_BPP8
    bpp = 1; break;
  default:
    bpp = 1; break; // conservative: assume 1 byte/pixel for unknown formats
  }
  return byte_stride / bpp;
}
} // namespace

bool RgaPreprocessor::rga_blit(const void *src_va, int src_w, int src_h,
                               int src_stride, int src_format,
                               int src_x, int src_y,
                               void *dst_va, int dst_w, int dst_h,
                               int dst_stride, int dst_format,
                               int dst_x, int dst_y) {
  rga_info_t src_info;
  std::memset(&src_info, 0, sizeof(src_info));
  src_info.fd = -1;
  src_info.virAddr = const_cast<void *>(src_va);
  src_info.mmuFlag = 1;
  rga_set_rect(&src_info.rect, src_x, src_y, src_w, src_h,
               to_pixel_stride(src_stride, src_format), src_h, src_format);

  rga_info_t dst_info;
  std::memset(&dst_info, 0, sizeof(dst_info));
  dst_info.fd = -1;
  dst_info.virAddr = dst_va;
  dst_info.mmuFlag = 1;
  rga_set_rect(&dst_info.rect, dst_x, dst_y, dst_w, dst_h,
               to_pixel_stride(dst_stride, dst_format), dst_h, dst_format);

  int ret = rga_lib_->RgaBlit(&src_info, &dst_info, nullptr);
  return ret == 0;
}

// -----------------------------------------------------------------------
// Startup self-check
// -----------------------------------------------------------------------
//
// We cannot assume the RGA hardware/driver on a given board produces correct
// output (permissions, "compatibility mode", kernel driver quirks, alignment).
// Rather than trust it blindly, at init time we run one RGB565->BGR888 blit on
// a synthetic image and compare against the trusted CPU path.  RGA only stays
// enabled if every pixel matches the CPU result within a small tolerance.
//
// Why a tolerance: RGA expands 5/6-bit channels to 8 bits via bit-replication,
// while the CPU path uses a plain left shift.  Those differ by a few LSBs, but
// the ordering/structure must still match.  A large or structural mismatch
// (wrong channel order, garbage, all-zero output) means RGA is unusable on this
// board and we must fall back to CPU.
bool RgaPreprocessor::self_check_rgb565_to_bgr() {
  // Build a synthetic RGB565 image with varied content across all channels.
  // Keep it 8-pixel-aligned width so the NEON CPU fast path is exercised too.
  const int w = 64;
  const int h = 16;
  const int stride = w * 2;
  std::vector<uint16_t> src(static_cast<std::size_t>(w) * h);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const unsigned r = static_cast<unsigned>(x) & 0x1F;        // 5 bits
      const unsigned g = static_cast<unsigned>(x + y) & 0x3F;    // 6 bits
      const unsigned b = static_cast<unsigned>(y * 2) & 0x1F;    // 5 bits
      src[static_cast<std::size_t>(y) * w + x] =
          static_cast<uint16_t>((r << 11) | (g << 5) | b);
    }
  }
  const auto *src_bytes = reinterpret_cast<const unsigned char *>(src.data());

  // Reference: CPU conversion.
  cv::Mat cpu_bgr;
  cpu_rgb565_to_bgr(cpu_bgr, src_bytes, w, h, stride);

  // Candidate: RGA conversion (direct blit, bypassing the public wrapper so a
  // failure here does not silently fall back and mask the problem).
  cv::Mat rga_bgr;
  rga_bgr.create(h, w, CV_8UC3);
  bool blit_ok;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    blit_ok = rga_blit(src_bytes, w, h, stride,
                       static_cast<int>(RK_FORMAT_RGB_565), 0, 0,
                       rga_bgr.data, w, h, static_cast<int>(w * 3),
                       static_cast<int>(RK_FORMAT_BGR_888), 0, 0);
  }
  if (!blit_ok)
    return false;

  // Compare with tolerance.  A per-channel delta up to 8 covers the
  // bit-replication vs shift difference; anything beyond that is a real bug.
  const int kTolerance = 8;
  for (int y = 0; y < h; ++y) {
    const auto *cpu_row = cpu_bgr.ptr<uint8_t>(y);
    const auto *rga_row = rga_bgr.ptr<uint8_t>(y);
    for (int i = 0; i < w * 3; ++i) {
      const int diff = std::abs(static_cast<int>(cpu_row[i]) -
                                static_cast<int>(rga_row[i]));
      if (diff > kTolerance)
        return false;
    }
  }
  return true;
}

// -----------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------

void RgaPreprocessor::rgb565_to_bgr(cv::Mat &bgr, const unsigned char *frame565,
                                    int width, int height, int stride_bytes) {
  bgr.create(height, width, CV_8UC3);

  if (!rga_available_) {
    cpu_rgb565_to_bgr(bgr, frame565, width, height, stride_bytes);
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  bool ok = rga_blit(
      frame565, width, height, stride_bytes,
      static_cast<int>(RK_FORMAT_RGB_565),
      0, 0,
      bgr.data, width, height, static_cast<int>(width * 3),
      static_cast<int>(RK_FORMAT_BGR_888),
      0, 0);

  if (!ok) {
    // RGA failed (e.g. alignment issue) — fall back to CPU.
    cpu_rgb565_to_bgr(bgr, frame565, width, height, stride_bytes);
  }
}

void RgaPreprocessor::resize(const cv::Mat &src, cv::Mat &dst,
                             int dst_w, int dst_h) {
  if (src.empty()) {
    dst = cv::Mat();
    return;
  }

  dst.create(dst_h, dst_w, src.type());

  if (!rga_available_) {
    cpu_resize(src, dst, dst_w, dst_h);
    return;
  }

  int format;
  int elem_size;
  switch (src.type()) {
  case CV_8UC3:
    format = static_cast<int>(RK_FORMAT_BGR_888);
    elem_size = 3;
    break;
  case CV_8UC1:
    format = static_cast<int>(RK_FORMAT_BPP8);
    elem_size = 1;
    break;
  default:
    cpu_resize(src, dst, dst_w, dst_h);
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  bool ok = rga_blit(
      src.data, src.cols, src.rows, static_cast<int>(src.cols * elem_size),
      format, 0, 0,
      dst.data, dst_w, dst_h, static_cast<int>(dst_w * elem_size),
      format, 0, 0);

  if (!ok)
    cpu_resize(src, dst, dst_w, dst_h);
}

void RgaPreprocessor::cvt_color(const cv::Mat &src, cv::Mat &dst,
                                int conversion_code) {
  if (src.empty()) {
    dst = cv::Mat();
    return;
  }

  // We only accelerate BGR↔RGB swaps; everything else goes to OpenCV.
  if (conversion_code != cv::COLOR_BGR2RGB &&
      conversion_code != cv::COLOR_RGB2BGR) {
    cpu_cvt_color(src, dst, conversion_code);
    return;
  }

  dst.create(src.rows, src.cols, CV_8UC3);

  if (!rga_available_) {
    cpu_cvt_color(src, dst, conversion_code);
    return;
  }

  // On RGA, BGR↔RGB is done by swapping the source format: read as BGR,
  // write as RGB (or vice-versa).  The actual pixel layout change is handled
  // by the blit engine's channel reordering.
  int src_fmt, dst_fmt;
  if (conversion_code == cv::COLOR_BGR2RGB) {
    src_fmt = static_cast<int>(RK_FORMAT_BGR_888);
    dst_fmt = static_cast<int>(RK_FORMAT_RGB_888);
  } else {
    src_fmt = static_cast<int>(RK_FORMAT_RGB_888);
    dst_fmt = static_cast<int>(RK_FORMAT_BGR_888);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const int stride = static_cast<int>(src.cols * 3);
  bool ok = rga_blit(src.data, src.cols, src.rows, stride, src_fmt, 0, 0,
                     dst.data, src.cols, src.rows, stride, dst_fmt, 0, 0);

  if (!ok)
    cpu_cvt_color(src, dst, conversion_code);
}

void RgaPreprocessor::crop_resize(const cv::Mat &src, cv::Mat &dst,
                                  const cv::Rect &roi,
                                  int dst_w, int dst_h) {
  if (src.empty() || roi.width <= 0 || roi.height <= 0) {
    dst = cv::Mat();
    return;
  }

  // Clamp ROI to source bounds.
  const int rx = std::max(0, roi.x);
  const int ry = std::max(0, roi.y);
  const int rw = std::min(roi.width, src.cols - rx);
  const int rh = std::min(roi.height, src.rows - ry);
  if (rw <= 0 || rh <= 0) {
    dst = cv::Mat();
    return;
  }

  dst.create(dst_h, dst_w, src.type());

  if (!rga_available_) {
    cpu_crop_resize(src, dst, cv::Rect(rx, ry, rw, rh), dst_w, dst_h);
    return;
  }

  int format;
  int elem_size;
  switch (src.type()) {
  case CV_8UC3:
    format = static_cast<int>(RK_FORMAT_BGR_888);
    elem_size = 3;
    break;
  case CV_8UC1:
    format = static_cast<int>(RK_FORMAT_BPP8);
    elem_size = 1;
    break;
  default:
    cpu_crop_resize(src, dst, cv::Rect(rx, ry, rw, rh), dst_w, dst_h);
    return;
  }

  // RGA crop+resize: set the source rect to the ROI region and the dest rect
  // to the full output dimensions.  RGA will sample only from the source rect.
  const int src_stride = static_cast<int>(src.cols * elem_size);
  std::lock_guard<std::mutex> lock(mutex_);
  bool ok = rga_blit(
      src.data, rw, rh, src_stride, format, rx, ry,
      dst.data, dst_w, dst_h, static_cast<int>(dst_w * elem_size),
      format, 0, 0);

  if (!ok)
    cpu_crop_resize(src, dst, cv::Rect(rx, ry, rw, rh), dst_w, dst_h);
}

void RgaPreprocessor::rgb565_to_rgb_resized(const unsigned char *frame565,
                                            int width, int height,
                                            int stride_bytes,
                                            cv::Mat &output,
                                            int dst_w, int dst_h) {
  output.create(dst_h, dst_w, CV_8UC3);

  if (!rga_available_) {
    // CPU fallback: 565→BGR → resize → BGR→RGB
    cv::Mat bgr;
    cpu_rgb565_to_bgr(bgr, frame565, width, height, stride_bytes);
    cv::Mat resized;
    cpu_resize(bgr, resized, dst_w, dst_h);
    cpu_cvt_color(resized, output, cv::COLOR_BGR2RGB);
    return;
  }

  // RGA combined: RGB565 source → RGB888 destination, with built-in resize.
  // RGA will do colour conversion (RGB565→RGB888) and bilinear downscale
  // in a single hardware pass.
  std::lock_guard<std::mutex> lock(mutex_);
  bool ok = rga_blit(
      frame565, width, height, stride_bytes,
      static_cast<int>(RK_FORMAT_RGB_565),
      0, 0,
      output.data, dst_w, dst_h, static_cast<int>(dst_w * 3),
      static_cast<int>(RK_FORMAT_RGB_888),
      0, 0);

  if (!ok) {
    cv::Mat bgr;
    cpu_rgb565_to_bgr(bgr, frame565, width, height, stride_bytes);
    cv::Mat resized;
    cpu_resize(bgr, resized, dst_w, dst_h);
    cpu_cvt_color(resized, output, cv::COLOR_BGR2RGB);
  }
}

} // namespace vision
