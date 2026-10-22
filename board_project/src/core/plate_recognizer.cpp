// 车牌识别器实现：YOLOv5 检测 + LPRNet 识别 两阶段流水线 (v2 新模型)
//
// 整体推理流程（per-frame）：
//
//   输入: RGB565 帧 (1280×720)
//     │
//     ▼
//   [vision::rgb565_to_bgr]  ← 颜色空间转换 (NEON SIMD)
//     │
//     ▼
//   [ztu::cv::resize]  ← 缩放到检测模型输入 (640×640)
//     │
//     ▼
//   [YTLOv5 NPU 推理]  ← YOLOv5 车牌检测
//     │
//     ▼
//   [CHW→HWC 转置]  ← 数据布局重排（适配后处理器）
//     │
//     ▼
//   [YoloV5Postprocessor::postprocess]  ← sigmoid 解码 + NMS
//     │
//     ▼
//   [坐标反算 + 裁剪]  ← 映射回原始帧坐标，过滤过小框
//     │
//     ▼
//   [对每个检测框]:
//     ├─ 裁剪 ROI → resize(168×48)  ← LPRNet v2 输入尺寸 (plate_recognition_color)
//     ├─ [LPRNet NPU 推理]  ← 字符识别 + 颜色分类
//     └─ [CTC 贪心解码]    ← logits → 车牌文本
//     │
//     ▼
//   [refine_plate_candidates]  ← 多维评分 + IoU 去重
//     │
//     ▼
//   输出: PlateFrameResult (车牌列表)

#include "core/plate_recognizer.h"

#include "common/plate_text_codec.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <filesystem>
#include <memory>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <QDebug>

#include "common/vision_common.h"
#include "inference/yolov5_postprocess.hpp"
#include "ztu/cv.hpp"
#include "ztu/rk/ez_rknn.hpp"

#include "core/plate_candidate_filter.h"
#include "inference/rknn_diag.hpp"

namespace {

// 车牌检测 YOLOv5 后处理配置：锚点参数
// 三层锚点：对应小/中/大尺度的车牌目标
const ztu::nn::YoloV5PostprocessConfig kPlatePostprocessConfig{
    {{10.0f, 13.0f, 16.0f, 30.0f, 33.0f, 23.0f},
     {30.0f, 61.0f, 62.0f, 45.0f, 59.0f, 119.0f},
     {116.0f, 90.0f, 156.0f, 198.0f, 373.0f, 326.0f}}};

// 模型颜色输出索引 → 颜色名（黑色/蓝色/绿色/白色/黄色）
const char *kPlateColorNames[] = {u8"黑", u8"蓝", u8"绿", u8"白", u8"黄"};

// 从模型颜色输出 [5] 解析颜色
std::string decode_plate_color(const float *color_logits, int n) {
  int best = 0;
  for (int i = 1; i < n; ++i)
    if (color_logits[i] > color_logits[best]) best = i;
  return kPlateColorNames[best];
}

// 根据车牌文本（不依赖颜色）判定特殊车牌类型
// 规则优先级：特殊字符 → 字符长度 → 默认
//
// 特殊字符判定（按优先级排列）：
//   学 → 教练车      警 → 警车        军 → 军车
//   使/领 → 使馆车   港/澳 → 港澳车   挂 → 货车
//   民/航 → 民航车   危 → 危化品车   险 → 保险车
//
// 长度判定（无特殊字符时，9+字符为新能源，7-8为标准车牌）
std::string classify_plate_type(const std::string &text) {
  if (text.empty()) return {};

  // UTF-8 字符串 → Unicode 码点数
  auto count_chars = [](const std::string &s) -> int {
    int n = 0;
    for (std::size_t i = 0; i < s.size(); ++n) {
      unsigned char c = static_cast<unsigned char>(s[i]);
      if (c < 0x80)      i += 1;
      else if ((c & 0xE0) == 0xC0) i += 2;
      else if ((c & 0xF0) == 0xE0) i += 3;
      else if ((c & 0xF8) == 0xF0) i += 4;
      else i += 1;
    }
    return n;
  };

  auto contains = [&text](const char *kw) -> bool {
    return text.find(kw) != std::string::npos;
  };

  // 特殊字符判定
  if (contains(u8"学")) return u8"教练车";
  if (contains(u8"警")) return u8"警车";
  if (contains(u8"军")) return u8"军车";
  if (contains(u8"使") || contains(u8"领")) return u8"使馆车";
  if (contains(u8"港") || contains(u8"澳")) return u8"港澳车";
  if (contains(u8"挂")) return u8"货车";
  if (contains(u8"民") || contains(u8"航")) return u8"民航车";
  if (contains(u8"危")) return u8"危化品车";
  if (contains(u8"险")) return u8"保险车";

  // 长度判定（无特殊字符时，9+字符为新能源，7-8为标准车牌）
  const int len = count_chars(text);
  if (len >= 9) return u8"新能源";
  if (len >= 7) return u8"标准车牌";
  if (len >= 5) return u8"短车牌";

  return {};
}

} // namespace

// PIMPL 实现：隐藏 RKNN 推理引擎和后处理器细节
class PlateRecognizer::Impl {
public:
  std::unique_ptr<ztu::rk::EzRknn> detector;   // YOLOv5 检测模型推理引擎
  std::unique_ptr<ztu::rk::EzRknn> recognizer; // LPRNet 识别模型推理引擎
  ztu::nn::YoloV5Postprocessor postprocessor{kPlatePostprocessConfig}; // YOLOv5 后处理器

  // 预分配图像 buffer（复用，减少 per-frame 堆分配）
  cv::Mat bgr;                                   // RGB565 → BGR 转换结果
  cv::Mat det_input;                              // 检测模型输入（缩放 + RGB 转换后）
  cv::Mat plate_crop;                             // 车牌 ROI 裁剪区域

  // 检测器输出 buffer：用于 CHW→HWC 数据布局重排
  // 外部 vector = 3 个检测头，内部 vector = 重排后的浮点数据
  std::vector<std::vector<float>> detector_output_buffers;
};

PlateRecognizer::PlateRecognizer() : impl_(new Impl()) {}

PlateRecognizer::~PlateRecognizer() { delete impl_; }

bool PlateRecognizer::initialize(const std::string &base_dir) {
  return ensure_models_loaded(base_dir);
}

// 加载 YOLOv5 检测模型和 LPRNet 识别模型
// 搜索顺序：环境变量 → 多级文件路径回退
//   - PANGO_PLATE_DET_MODEL → plate_yolov5.rknn / plate_detect.rknn
//   - PANGO_PLATE_REC_MODEL  → plate_lprnet.rknn / lprnet.rknn
bool PlateRecognizer::ensure_models_loaded(const std::string &base_dir) {
  if (initialized_) {
    return true;
  }

  const std::filesystem::path root(base_dir);
  const std::filesystem::path parent = root.parent_path();

  // 检测模型路径解析
  detector_model_path_ = vision::read_env_path("PANGO_PLATE_DET_MODEL");
  if (detector_model_path_.empty()) {
    detector_model_path_ = vision::find_first_existing_path({
        root / "models" / "plate_yolov5.rknn",
        root / "plate_yolov5.rknn",
        parent / "models" / "plate_yolov5.rknn",
        parent / "plate_yolov5.rknn",
        root / "models" / "plate_detect.rknn",
        root / "plate_detect.rknn",
        parent / "models" / "plate_detect.rknn",
        parent / "plate_detect.rknn",
    });
  }

  // 识别模型路径解析
  recognizer_model_path_ = vision::read_env_path("PANGO_PLATE_REC_MODEL");
  if (recognizer_model_path_.empty()) {
    recognizer_model_path_ = vision::find_first_existing_path({
        root / "models" / "plate_lprnet.rknn",
        root / "plate_lprnet.rknn",
        parent / "models" / "plate_lprnet.rknn",
        parent / "plate_lprnet.rknn",
        root / "models" / "lprnet.rknn",
        root / "lprnet.rknn",
        parent / "models" / "lprnet.rknn",
        parent / "lprnet.rknn",
    });
  }

  if (detector_model_path_.empty() || recognizer_model_path_.empty()) {
    last_error_ =
        "plate RKNN models not found, expected plate_yolov5.rknn and "
        "plate_lprnet.rknn";
    return false;
  }

  // 创建 RKNN 推理引擎（均使用 CHW2HWC 布局以满足 NPU 要求）
  try {
    impl_->detector = std::make_unique<ztu::rk::EzRknn>(
        detector_model_path_, ztu::rk::EzRknn::Layout::CHW2HWC);
    impl_->recognizer = std::make_unique<ztu::rk::EzRknn>(
        recognizer_model_path_, ztu::rk::EzRknn::Layout::CHW2HWC);
  } catch (const std::exception &ex) {
    last_error_ = ex.what();
    impl_->detector.reset();
    impl_->recognizer.reset();
    return false;
  }

  // 一次性输出两个模型的张量属性，便于在真机上确认检测头输出布局
  // （5D 需转置 / 4D 无需转置），不影响推理行为。
  vision::log_rknn_output_layout("PLATE-DET", *impl_->detector);
  vision::log_rknn_output_layout("PLATE-REC", *impl_->recognizer);

  initialized_ = true;
  last_error_.clear();
  return true;
}

// CTC 贪心解码：将 LPRNet 输出 logits 转换为车牌文本
//
// 支持两种输出布局（自动适配）：
//   time_first=false: logits[c * time_steps + t]  ← 旧 LPRNet [classes, time_steps]
//   time_first=true:  logits[t * class_count + c] ← 新 plate_recognition_color [time_steps, classes]
std::string PlateRecognizer::decode_plate_text(const float *logits,
                                               int class_count,
                                               int time_steps,
                                               bool time_first) const {
  // 对每个时间步取 argmax
  std::vector<int> indices;
  indices.reserve(static_cast<std::size_t>(time_steps));
  for (int t = 0; t < time_steps; ++t) {
    int best_class = 0;
    float best_score = time_first ? logits[t * class_count] : logits[t];
    for (int c = 1; c < class_count; ++c) {
      const float score = time_first
          ? logits[t * class_count + c]
          : logits[c * time_steps + t];
      if (score > best_score) {
        best_score = score;
        best_class = c;
      }
    }
    indices.push_back(best_class);
  }
  return lpr::decode_plate_indices(indices);
}

// 主推理函数：对一帧 RGB565 图像执行车牌检测 + 识别
//
// 参数：
//   frame565: 原始 RGB565 数据
//   width/height: 原始帧宽高（通常 1280×720）
//   stride_bytes: 行步长（通常 width*2）
//
// 返回值：PlateFrameResult 包含所有检测到的车牌（位置 + 文本）
PlateFrameResult PlateRecognizer::infer_rgb565(const unsigned char *frame565,
                                               int width, int height,
                                               int stride_bytes) {
  PlateFrameResult frame_result;

  // 模型未加载
  if (!initialized_) {
    frame_result.error_message = last_error_;
    return frame_result;
  }

  // 参数合法性检查
  if (frame565 == nullptr || width <= 0 || height <= 0 ||
      stride_bytes < width * 2) {
    frame_result.error_message = "invalid frame buffer";
    return frame_result;
  }

  try {
    using clock = std::chrono::steady_clock;
    const auto t_total_0 = clock::now();

    // ===== 阶段 1：帧预处理 =====

    // RGB565 → BGR（NEON SIMD 加速，< 1ms）
    vision::rgb565_to_bgr(impl_->bgr, frame565, width, height, stride_bytes);

    // 缩放到检测模型输入尺寸（通常 640×640）
    const int det_model_w = impl_->detector->input_attrs[0].dims[2];
    const int det_model_h = impl_->detector->input_attrs[0].dims[1];
    const auto resize_param =
        ztu::cv::calculateResize(impl_->bgr.cols, impl_->bgr.rows,
                                 det_model_w, det_model_h);
    impl_->det_input = ztu::cv::resize(impl_->bgr, resize_param);

    // BGR → RGB（模型输入格式要求）
    cv::cvtColor(impl_->det_input, impl_->det_input, cv::COLOR_BGR2RGB);

    const auto t_pre = clock::now();

    // ===== 阶段 2：YOLOv5 车牌检测 =====

    auto det_outputs = impl_->detector->inference(impl_->det_input.data);
    if (det_outputs.size() < 3) {
      frame_result.error_message = "unexpected detector output count";
      return frame_result;
    }

    const auto t_det_infer = clock::now();

    // 输出按空间尺寸降序排列（大特征图 → 小特征图）
    // 这确保后处理器的 3 个输入对应 stride 8/16/32 的正确顺序
    std::vector<int> output_order(det_outputs.size());
    for (std::size_t i = 0; i < det_outputs.size(); ++i) {
      output_order[i] = static_cast<int>(i);
    }
    std::sort(output_order.begin(), output_order.end(),
              [&](int lhs, int rhs) {
                const auto &lhs_attr =
                    impl_->detector->output_attrs[static_cast<std::size_t>(lhs)];
                const auto &rhs_attr =
                    impl_->detector->output_attrs[static_cast<std::size_t>(rhs)];
                // 提取空间维度：模型输出为 [batch, anchors, H, W, props] 或 [1, anchors*H*W, props]
                const int lhs_h = lhs_attr.n_dims >= 4
                                      ? static_cast<int>(lhs_attr.dims[lhs_attr.n_dims - 3])
                                      : 1;
                const int lhs_w = lhs_attr.n_dims >= 4
                                      ? static_cast<int>(lhs_attr.dims[lhs_attr.n_dims - 2])
                                      : 1;
                const int rhs_h = rhs_attr.n_dims >= 4
                                      ? static_cast<int>(rhs_attr.dims[rhs_attr.n_dims - 3])
                                      : 1;
                const int rhs_w = rhs_attr.n_dims >= 4
                                      ? static_cast<int>(rhs_attr.dims[rhs_attr.n_dims - 2])
                                      : 1;
                return lhs_h * lhs_w > rhs_h * rhs_w; // 按空间尺寸降序
              });

    // CHW → HWC 数据布局转置（RKNN 输出为 CHW，后处理器期望 HWC）
    //
    // CHW: data[((anchor * H) + row) * W + col) * prop_size + prop]
    // HWC: data[((anchor * prop_size + prop) * grid_len) + row * W + col]
    const float *output_ptrs[3] = {};
    for (int ordered_index = 0; ordered_index < 3; ++ordered_index) {
      const auto output_index =
          static_cast<std::size_t>(output_order[ordered_index]);
      const auto &attr = impl_->detector->output_attrs[output_index];

      // 处理 4D 和 5D 输出的维度偏移
      const int dims_index_offset = attr.n_dims >= 5 ? 0 : -1;
      const int anchor_count =
          attr.n_dims >= 5 ? static_cast<int>(attr.dims[1]) : 3;
      const int grid_h =
          static_cast<int>(attr.dims[attr.n_dims - 3 + dims_index_offset]);
      const int grid_w =
          static_cast<int>(attr.dims[attr.n_dims - 2 + dims_index_offset]);
      const int prop_size = static_cast<int>(attr.dims[attr.n_dims - 1]);
      const int grid_len = grid_h * grid_w;
      const auto buffer_size =
          static_cast<std::size_t>(anchor_count * prop_size * grid_len);

      // 确保输出 buffer 已分配
      if (impl_->detector_output_buffers.size() < 3) {
        impl_->detector_output_buffers.resize(3);
      }
      impl_->detector_output_buffers[static_cast<std::size_t>(ordered_index)]
          .assign(buffer_size, 0.0f);

      // 执行转置
      const float *src = det_outputs[output_index];
      for (int anchor_index = 0; anchor_index < anchor_count; ++anchor_index) {
        for (int row = 0; row < grid_h; ++row) {
          for (int col = 0; col < grid_w; ++col) {
            for (int prop_index = 0; prop_index < prop_size; ++prop_index) {
              const std::size_t src_index = static_cast<std::size_t>(
                  (((anchor_index * grid_h) + row) * grid_w + col) * prop_size +
                  prop_index);
              const std::size_t dst_index = static_cast<std::size_t>(
                  ((anchor_index * prop_size + prop_index) * grid_len) +
                  row * grid_w + col);
              impl_->detector_output_buffers[static_cast<std::size_t>(
                  ordered_index)][dst_index] = src[src_index];
            }
          }
        }
      }
      output_ptrs[ordered_index] =
          impl_->detector_output_buffers[static_cast<std::size_t>(ordered_index)]
              .data();
    }

    // YOLOv5 后处理：sigmoid 解码 + NMS
    // 阈值：conf_threshold=0.40, nms_threshold=0.50, num_classes=1（仅车牌类）
    ztu::nn::object_detect_result_list detections;
    impl_->postprocessor.postprocess(output_ptrs, 0.40f, 0.50f,
                                      &detections, det_model_w, det_model_h, 1);
    const auto t_post = clock::now();

    // ===== 阶段 3：对每个检测框进行 LPRNet 字符识别 =====

    for (auto &det : detections.results) {
      // 坐标反算：从模型空间映射回原始帧空间
      const auto mapped_top_left =
          ztu::cv::reverseMapPoint(det.box.left, det.box.top, resize_param);
      const auto mapped_bottom_right = ztu::cv::reverseMapPoint(
          det.box.right, det.box.bottom, resize_param);
      det.box.left = mapped_top_left.first;
      det.box.top = mapped_top_left.second;
      det.box.right = mapped_bottom_right.first;
      det.box.bottom = mapped_bottom_right.second;

      // 裁剪到帧边界并过滤过小的检测框
      const cv::Rect detect_roi = vision::clip_rect_to_frame(det.box, width, height);
      if (detect_roi.width <= 2 || detect_roi.height <= 2) {
        continue;
      }

      const cv::Rect clipped_recognition_roi = detect_roi;
      if (clipped_recognition_roi.width <= 2 ||
          clipped_recognition_roi.height <= 2) {
        continue;
      }

      // 裁剪车牌 ROI 并缩放到 LPRNet 输入尺寸（v2: 168×48, plate_recognition_color）
      impl_->plate_crop = impl_->bgr(clipped_recognition_roi).clone();
      cv::resize(impl_->plate_crop, impl_->plate_crop, cv::Size(168, 48));

      // LPRNet 推理
      auto rec_outputs = impl_->recognizer->inference(impl_->plate_crop.data);
      if (rec_outputs.empty()) {
        continue;
      }

      // 解析 LPRNet 输出形状 — 自动检测 time_first 布局
      // 旧模型: [batch, classes, time_steps] → dims[-2]=class_count
      // 新模型: [batch, time_steps, classes] → dims[-1]=class_count (匹配 plate_charset 大小)
      const auto &attr = impl_->recognizer->output_attrs[0];
      const int charset_sz = static_cast<int>(lpr::plate_charset().size());
      int class_count = charset_sz;
      int time_steps = 0;
      bool time_first = false; // 默认 [classes, time_steps] 布局
      if (attr.n_dims >= 3) {
        const int d1 = static_cast<int>(attr.dims[attr.n_dims - 2]);
        const int d2 = static_cast<int>(attr.dims[attr.n_dims - 1]);
        if (d2 == charset_sz) {
          // 新模型: [time_steps, classes] 布局
          class_count = d2;
          time_steps = d1;
          time_first = true;
        } else {
          // 旧模型: [classes, time_steps] 布局
          class_count = d1;
          time_steps = d2;
        }
      } else if (attr.n_dims == 2) {
        class_count = static_cast<int>(attr.dims[0]);
        time_steps = static_cast<int>(attr.dims[1]);
      } else if (charset_sz > 0) {
        time_steps = static_cast<int>(attr.n_elems) / charset_sz;
      }
      if (time_steps <= 0 || class_count <= 0) {
        continue;
      }

      // CTC 贪心解码 + 颜色解析 + 类型分类
      PlateDetectionResult result;
      result.left = detect_roi.x;
      result.top = detect_roi.y;
      result.right = detect_roi.x + detect_roi.width;
      result.bottom = detect_roi.y + detect_roi.height;
      result.confidence = det.prop;
      result.text = decode_plate_text(rec_outputs[0], class_count, time_steps, time_first);

      // 解析车牌颜色（模型输出 [5]，仅当模型有双输出时）
      if (impl_->recognizer->output_attrs.size() >= 2) {
        result.color = decode_plate_color(rec_outputs[1], 5);
      }
      // 根据文本规则分类车牌类型（不依赖颜色）
      result.plate_type = classify_plate_type(result.text);

      frame_result.plates.push_back(result);
    }

    const auto t_lpr = clock::now();

    // ===== 阶段 4：候选精炼 =====
    // 多维评分 + IoU 去重，去除低质量和重叠的车牌候选
    lpr::refine_plate_candidates(&frame_result.plates);

    const auto t_refine = clock::now();

    // 计时输出：每 30 帧打印一次，避免刷屏
    {
      static int log_skip = 0;
      if (++log_skip >= 30) {
        log_skip = 0;
        using ms = std::chrono::duration<double, std::milli>;
        qDebug().nospace()
            << "[PLATE] total:" << ms(t_refine - t_total_0).count()
            << "ms | pre:" << ms(t_pre - t_total_0).count()
            << "ms | det:" << ms(t_det_infer - t_pre).count()
            << "ms | post:" << ms(t_post - t_det_infer).count()
            << "ms | lpr:" << ms(t_lpr - t_post).count()
            << "ms | refine:" << ms(t_refine - t_lpr).count()
            << "ms | plates:" << frame_result.plates.size();
      }
    }

    frame_result.model_ready = true;
    return frame_result;
  } catch (const std::exception &ex) {
    frame_result.error_message = ex.what();
    return frame_result;
  }
}
