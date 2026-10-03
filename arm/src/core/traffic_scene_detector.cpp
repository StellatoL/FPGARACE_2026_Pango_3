// 交通场景检测器实现
//
// 使用 YOLOv5 模型检测 6 类交通元素：
//   0: 斑马线 (zebra) - 红框     1: 行人 (person) - 橙框
//   2: 红灯 (red)    - 红框     3: 绿灯 (green)  - 绿框
//   4: 黑灯 (black)  - 黑框     5: 黄灯 (yellow) - 黄框
//
// 推理流程：
// 1. RGB565 → BGR（NEON SIMD 加速）
// 2. 缩放到 YOLOv5 模型输入尺寸 → NPU 推理
// 3. 统一 YOLOv5 后处理器（NMS + 坐标解码）
// 4. 坐标反算 → 裁剪到帧边界
// 5. 调用交通规则引擎（traffic_scene_rules）分析告警条件
// 6. 构建包含检测框叠加层和告警标记的 VisionFrameResult

#include "core/traffic_scene_detector.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <QDebug>

#include "core/traffic_scene_rules.h"
#include "inference/rknn_diag.hpp"
#include "inference/sort_tracker.h"
#include "common/vision_common.h"
#include "inference/yolov5_postprocess.hpp"
#include "ztu/cv.hpp"
#include "ztu/nn/structures.hpp"
#include "ztu/rk/ez_rknn.hpp"

namespace {

// 交通场景检测类别定义：名称 + 检测框颜色（BGR 格式）
struct TrafficClassStyle {
  const char *name; // 类别名称
  int color_r;      // 检测框颜色 R 分量
  int color_g;      // 检测框颜色 G 分量
  int color_b;      // 检测框颜色 B 分量
};

const TrafficClassStyle kTrafficClasses[] = {
    {"zebra", 0, 0, 255},      // 0: 斑马线 — 红色框
    {"person", 255, 165, 0},   // 1: 行人 — 橙色框
    {"red", 255, 0, 0},        // 2: 红灯 — 红色框
    {"green", 0, 255, 0},      // 3: 绿灯 — 绿色框
    {"black", 0, 0, 0},        // 4: 黑灯 — 黑色框
    {"yellow", 255, 255, 0},   // 5: 黄灯 — 黄色框
};

// COCO 类别名（yolov5s_traffic 副模型用）
const char *kCocoNames[] = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard",
    "sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
    "donut","cake","chair","couch","potted plant","bed","dining table","toilet",
    "tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush"
};

// Fast postprocessor for yolov5s_traffic [1,N,85] flat format
static void postprocess_flat_v2(
    const std::vector<float *> &outputs,
    float conf_thresh, float nms_thresh,
    ztu::nn::object_detect_result_list *results,
    int model_w, int model_h)
{
  results->results.clear();
  const int grid[3][2] = {{80,80},{40,40},{20,20}};

  struct Det { float x1,y1,x2,y2,score; int cls; };
  std::vector<Det> dets;

  auto sig = [](float x) {
    if (x < -20) return 0.f; if (x > 20) return 1.f;
    return 1.f / (1.f + std::exp(-x));
  };

  for (int h = 0; h < 3; ++h) {
    const int total = 3 * grid[h][0] * grid[h][1];
    const float *d = outputs[h];
    for (int i = 0; i < total; ++i) {
      // ONNX export already applied sigmoid — use raw values directly
      float obj = d[i*85+4];
      if (obj < conf_thresh) continue;
      float best=0; int bc=0;
      for (int k : {2, 5, 7}) {
        float s = d[i*85+5+k];
        if (s > best) { best = s; bc = k; }
      }
      if (best < conf_thresh) continue;
      // cx,cy,w,h are already in pixel space (decoded by ONNX)
      float cx = d[i*85], cy = d[i*85+1], w = d[i*85+2], hh = d[i*85+3];
      dets.push_back({cx-w/2, cy-hh/2, cx+w/2, cy+hh/2, obj*best, bc});
    }
  }
  if (dets.empty()) return;

  std::sort(dets.begin(), dets.end(), [](auto &a, auto &b){ return a.score > b.score; });
  std::vector<bool> sup(dets.size(), false);
  for (size_t i = 0; i < dets.size(); ++i) {
    if (sup[i]) continue;
    float a = (dets[i].x2-dets[i].x1)*(dets[i].y2-dets[i].y1);
    for (size_t j = i+1; j < dets.size(); ++j) {
      if (sup[j] || dets[i].cls != dets[j].cls) continue;
      float ix1 = std::max(dets[i].x1, dets[j].x1);
      float iy1 = std::max(dets[i].y1, dets[j].y1);
      float ix2 = std::min(dets[i].x2, dets[j].x2);
      float iy2 = std::min(dets[i].y2, dets[j].y2);
      float iw = std::max(0.f, ix2-ix1), ih = std::max(0.f, iy2-iy1);
      float ia = iw*ih;
      float ua = a + (dets[j].x2-dets[j].x1)*(dets[j].y2-dets[j].y1) - ia;
      if (ua > 0 && ia/ua > nms_thresh) sup[j] = true;
    }
  }
  for (size_t i = 0; i < dets.size(); ++i) {
    if (sup[i]) continue;
    ztu::nn::object_detect_result r;
    r.box.left = (int)std::clamp(dets[i].x1, 0.f, (float)model_w);
    r.box.top = (int)std::clamp(dets[i].y1, 0.f, (float)model_h);
    r.box.right = (int)std::clamp(dets[i].x2, 0.f, (float)model_w);
    r.box.bottom = (int)std::clamp(dets[i].y2, 0.f, (float)model_h);
    r.prop = dets[i].score; r.cls_id = dets[i].cls;
    results->results.push_back(r);
  }
  results->count = results->results.size();
}

// 交通场景 YOLOv5 后处理配置：锚点参数（从训练模型导出）
// 三层锚点：小目标 / 中目标 / 大目标
const ztu::nn::YoloV5PostprocessConfig kTrafficPostprocessConfig{
    {{5.64062f, 12.39062f, 7.85547f, 18.5f, 10.71875f, 28.14062f},
     {63.5f, 9.07812f, 15.92188f, 41.3125f, 25.48438f, 69.9375f},
     {218.375f, 16.32812f, 49.90625f, 100.3125f, 612.5f, 90.0f}}};

// 生成检测框显示的文本标签（如 "zebra 85%"）
std::string detection_label(int class_id, float confidence) {
  std::ostringstream oss;
  if (class_id >= 0 &&
      class_id <
          static_cast<int>(sizeof(kTrafficClasses) / sizeof(kTrafficClasses[0]))) {
    oss << kTrafficClasses[class_id].name;
  } else {
    oss << "obj"; // 未知类别
  }
  oss << " " << static_cast<int>(confidence * 100.0f) << "%";
  return oss.str();
}

// 将检测结果转换为叠加层信息（用于 QPainter 绘制）
OverlayBoxResult make_overlay_box(const TrafficSceneDetection &detection) {
  OverlayBoxResult overlay;
  overlay.left = detection.left;
  overlay.top = detection.top;
  overlay.right = detection.right;
  overlay.bottom = detection.bottom;
  overlay.confidence = detection.confidence;
  overlay.label = detection_label(detection.class_id, detection.confidence);

  // 根据类别设置检测框颜色
  if (detection.class_id >= 0 &&
      detection.class_id <
          static_cast<int>(sizeof(kTrafficClasses) / sizeof(kTrafficClasses[0]))) {
    const auto &style = kTrafficClasses[detection.class_id];
    overlay.color_r = style.color_r;
    overlay.color_g = style.color_g;
    overlay.color_b = style.color_b;
  }

  return overlay;
}

} // namespace

// PIMPL 实现：隐藏 RKNN 推理引擎和后处理器细节
class TrafficSceneDetector::Impl {
public:
  std::unique_ptr<ztu::rk::EzRknn> detector;                // 主: traffic_yolov5 (6类)
  std::unique_ptr<ztu::rk::EzRknn> vehicle_detector;       // 副: yolov5s_traffic (可选)
  ztu::nn::YoloV5Postprocessor postprocessor{kTrafficPostprocessConfig};
  cv::Mat bgr; cv::Mat det_input;
  bool has_vehicle_model = false;

  sort::Sort person_tracker{3, 3, 0.3f};
  sort::Sort vehicle_tracker{3, 3, 0.3f};
  int frame_count = 0;
  float lane_vx = 0, lane_vy = 1;
  int lane_samples = 0;

  // ===== ADAS 感知扩展：最近一帧的缓存（build_adas_perception 用）=====
  std::vector<sort::Sort::Track> last_person_tracks;   // 最近一帧行人轨迹
  std::vector<sort::Sort::Track> last_vehicle_tracks; // 最近一帧车辆轨迹
  TrafficSceneAnalysis last_analysis;                  // 最近一帧规则分析结果
  std::vector<TrafficSceneDetection> last_detections; // 最近一帧检测（灯框选取用）
  int last_frame_w = 0, last_frame_h = 0;              // 最近一帧尺寸
};

TrafficSceneDetector::TrafficSceneDetector() : impl_(new Impl()) {}

TrafficSceneDetector::~TrafficSceneDetector() { delete impl_; }

// 最近一帧的 BGR 图像（车道线/导航箭头分析的输入）
const cv::Mat &TrafficSceneDetector::last_bgr() const { return impl_->bgr; }

bool TrafficSceneDetector::initialize(const std::string &base_dir) {
  return ensure_model_loaded(base_dir);
}

// 加载交通场景检测模型
// 搜索顺序：环境变量 PANGO_TRAFFIC_DET_MODEL → 多级文件路径回退
bool TrafficSceneDetector::ensure_model_loaded(const std::string &base_dir) {
  if (initialized_) {
    return true;
  }

  const std::filesystem::path root(base_dir);
  const std::filesystem::path parent = root.parent_path();

  // 优先使用环境变量指定路径
  detector_model_path_ = vision::read_env_path("PANGO_TRAFFIC_DET_MODEL");
  if (detector_model_path_.empty()) {
    // 多级回退搜索：当前目录 → 上级目录 → models/ 子目录
    detector_model_path_ = vision::find_first_existing_path({
        root / "models" / "traffic_yolov5.rknn",
        root / "traffic_yolov5.rknn",
        parent / "models" / "traffic_yolov5.rknn",
        parent / "traffic_yolov5.rknn",
        root / "models" / "best_rk3568.rknn",
        root / "best_rk3568.rknn",
        parent / "models" / "best_rk3568.rknn",
        parent / "best_rk3568.rknn",
    });
  }

  if (detector_model_path_.empty()) {
    last_error_ =
        "traffic RKNN model not found, expected traffic_yolov5.rknn";
    return false;
  }

  try {
    // 创建 RKNN 推理引擎，CHW2HWC 布局（NPU 要求）
    impl_->detector = std::make_unique<ztu::rk::EzRknn>(
        detector_model_path_, ztu::rk::EzRknn::Layout::CHW2HWC);
    // 一次性诊断：打印检测头输出张量维度，用于确认无需转置的前提
    // （traffic 路径直接把 inference() 输出喂给后处理器，要求其为 4D
    //  [1, na*prop, H, W] 的原生 NCHW 布局；若为 5D 则需转置，届时会在日志中暴露）
    vision::log_rknn_output_layout("TRAFFIC-DET", *impl_->detector);
  } catch (const std::exception &ex) {
    last_error_ = ex.what();
    impl_->detector.reset();
    return false;
  }

  initialized_ = true;
  last_error_.clear();

  // 尝试加载车辆副模型（失败不阻塞）
  {
    std::string veh = vision::find_first_existing_path({
        root / "models" / "yolov5s_traffic.rknn",
        root / "yolov5s_traffic.rknn"});
    if (!veh.empty()) {
      try {
        impl_->vehicle_detector = std::make_unique<ztu::rk::EzRknn>(
            veh, ztu::rk::EzRknn::Layout::CHW2HWC);
        impl_->has_vehicle_model = true;
        vision::log_rknn_output_layout("TRAFFIC-VEH", *impl_->vehicle_detector);
      } catch (...) { impl_->has_vehicle_model = false; }
    }
  }
  return true;
}

VisionFrameResult TrafficSceneDetector::infer_rgb565(const unsigned char *frame565,
                                                     int width, int height,
                                                     int stride_bytes) {
  VisionFrameResult frame_result;
  frame_result.mode = DetectionMode::TrafficScene;

  // 模型未就绪
  if (!initialized_) {
    frame_result.error_message = last_error_;
    return frame_result;
  }

  // 帧参数合法性校验
  if (frame565 == nullptr || width <= 0 || height <= 0 ||
      stride_bytes < width * 2) {
    frame_result.error_message = "invalid frame buffer";
    return frame_result;
  }

  frame_result.model_ready = true;

  try {
    using clock = std::chrono::steady_clock;
    const auto t_total_0 = clock::now();

    // 步骤 1：RGB565 → BGR 颜色空间转换（NEON SIMD 加速）
    vision::rgb565_to_bgr(impl_->bgr, frame565, width, height, stride_bytes);

    // 步骤 2：缩放 + 颜色空间转换 BGR→RGB（模型输入要求）
    const int model_w = impl_->detector->input_attrs[0].dims[2];
    const int model_h = impl_->detector->input_attrs[0].dims[1];
    const auto resize_param =
        ztu::cv::calculateResize(impl_->bgr.cols, impl_->bgr.rows,
                                 model_w, model_h);
    impl_->det_input = ztu::cv::resize(impl_->bgr, resize_param);
    cv::cvtColor(impl_->det_input, impl_->det_input, cv::COLOR_BGR2RGB);

    const auto t_pre = clock::now();

    // 步骤 3：NPU 推理
    auto detector_outputs = impl_->detector->inference(impl_->det_input.data);
    if (detector_outputs.size() < 3) {
      frame_result.error_message = "unexpected detector output count";
      frame_result.model_ready = false;
      return frame_result;
    }

    const auto t_infer = clock::now();

    // 步骤 4：后处理 — 自动检测输出格式
    ztu::nn::object_detect_result_list detections;
    const auto &o0 = impl_->detector->output_attrs[0];
    const bool flat = (o0.n_dims == 3 && o0.dims[1] > 1000);

    if (flat) {
      std::vector<float *> ptrs;
      for (auto *p : detector_outputs) ptrs.push_back(p);
      postprocess_flat_v2(ptrs, 0.25f, 0.40f, &detections, model_w, model_h);
    } else {
      std::vector<const float *> ptrs;
      for (auto *p : detector_outputs) ptrs.push_back(p);
      impl_->postprocessor.postprocess(ptrs.data(), 0.25f, 0.40f, &detections,
          model_w, model_h, (int)(sizeof(kTrafficClasses)/sizeof(kTrafficClasses[0])));
    }

    const auto t_post = clock::now();

    // 步骤 5：坐标反算 + 裁剪 + 构建结果
    std::vector<TrafficSceneDetection> scene_detections;
    for (auto &det : detections.results) {
      // 坐标从模型空间反算到原始帧空间
      const auto mapped_top_left =
          ztu::cv::reverseMapPoint(det.box.left, det.box.top, resize_param);
      const auto mapped_bottom_right = ztu::cv::reverseMapPoint(
          det.box.right, det.box.bottom, resize_param);
      det.box.left = mapped_top_left.first;
      det.box.top = mapped_top_left.second;
      det.box.right = mapped_bottom_right.first;
      det.box.bottom = mapped_bottom_right.second;

      // 裁剪到帧边界，过滤过小的检测框
      const cv::Rect clipped = vision::clip_rect_to_frame(det.box, width, height);
      if (clipped.width <= 1 || clipped.height <= 1) {
        continue;
      }

      TrafficSceneDetection scene_detection;
      scene_detection.class_id = det.cls_id;
      scene_detection.left = clipped.x;
      scene_detection.top = clipped.y;
      scene_detection.right = clipped.x + clipped.width;
      scene_detection.bottom = clipped.y + clipped.height;
      scene_detection.confidence = det.prop;
      scene_detections.push_back(scene_detection);
      frame_result.overlays.push_back(make_overlay_box(scene_detection));
    }

    // ===== 车辆副模型检测（可选）=====
    if (impl_->has_vehicle_model) {
      try {
        auto vout = impl_->vehicle_detector->inference(impl_->det_input.data);
        if (vout.size() >= 3) {
          ztu::nn::object_detect_result_list vdet;
          std::vector<float *> vp;
          for (auto *p : vout) vp.push_back(p);
          postprocess_flat_v2(vp, 0.55f, 0.50f, &vdet, model_w, model_h);
          for (auto &vd : vdet.results) {
            if (vd.cls_id != 2 && vd.cls_id != 5 && vd.cls_id != 7) continue;
            auto mt = ztu::cv::reverseMapPoint(vd.box.left, vd.box.top, resize_param);
            auto mb = ztu::cv::reverseMapPoint(vd.box.right, vd.box.bottom, resize_param);
            auto c = vision::clip_rect_to_frame({mt.first, mt.second, mb.first, mb.second}, width, height);
            if (c.width <= 1 || c.height <= 1) continue;
            TrafficSceneDetection vs;
            vs.class_id = 10; vs.left = c.x; vs.top = c.y;
            vs.right = c.x + c.width; vs.bottom = c.y + c.height;
            vs.confidence = vd.prop;
            scene_detections.push_back(vs);
            OverlayBoxResult vb;
            vb.left = vs.left; vb.top = vs.top; vb.right = vs.right; vb.bottom = vs.bottom;
            vb.confidence = vd.prop;
            vb.color_r = 0; vb.color_g = 100; vb.color_b = 255;
            vb.label = std::string(kCocoNames[vd.cls_id]) + " " +
                       std::to_string((int)(vd.prop * 100)) + "%";
            frame_result.overlays.push_back(vb);
          }
        }
      } catch (...) {}
    }

    // ===== 交通规则分析 =====
    const TrafficSceneAnalysis &a = analyze_traffic_scene(scene_detections);

    // A: 绿灯通行
    if (a.person_on_zebra) {
      int zw = a.zebra_detection.right - a.zebra_detection.left;
      int zh = a.zebra_detection.bottom - a.zebra_detection.top;
      frame_result.alert_marker.visible = true;
      frame_result.alert_marker.center_x = a.zebra_detection.left + zw/2;
      frame_result.alert_marker.center_y = a.zebra_detection.top + zh/2;
      frame_result.alert_marker.radius = std::max(12, std::min(zw,zh)/3);
      frame_result.alert_marker.text = "P";
      frame_result.status_text = "Pedestrian on zebra — green light active";
    }
    // B: 行人闯红灯
    if (a.person_on_zebra_red) {
      int zw = a.zebra_detection.right - a.zebra_detection.left;
      int zh = a.zebra_detection.bottom - a.zebra_detection.top;
      frame_result.alert_marker.visible = true;
      frame_result.alert_marker.center_x = a.zebra_detection.left + zw/2;
      frame_result.alert_marker.center_y = a.zebra_detection.top + zh/2;
      frame_result.alert_marker.fill_r = 255; frame_result.alert_marker.fill_g = 0; frame_result.alert_marker.fill_b = 0;
      frame_result.alert_marker.stroke_r = 255; frame_result.alert_marker.stroke_g = 255; frame_result.alert_marker.stroke_b = 0;
      frame_result.alert_marker.radius = std::max(14, std::min(zw,zh)/2);
      frame_result.alert_marker.text = "!";
      frame_result.status_text = "VIOLATION: Pedestrian crossing on RED light!";
    }
    // C: 车辆闯红灯
    if (a.vehicle_on_zebra_red) {
      int zw = a.zebra_detection.right - a.zebra_detection.left;
      int zh = a.zebra_detection.bottom - a.zebra_detection.top;
      frame_result.alert_marker.visible = true;
      frame_result.alert_marker.center_x = a.zebra_detection.left + zw/2;
      frame_result.alert_marker.center_y = a.zebra_detection.top + zh/2;
      frame_result.alert_marker.fill_r = 255; frame_result.alert_marker.fill_g = 0; frame_result.alert_marker.fill_b = 0;
      frame_result.alert_marker.stroke_r = 255; frame_result.alert_marker.stroke_g = 255; frame_result.alert_marker.stroke_b = 0;
      frame_result.alert_marker.radius = std::max(14, std::min(zw,zh)/2);
      frame_result.alert_marker.text = "X";
      frame_result.status_text = "VIOLATION: Vehicle running RED light!";
    }

    // ===== SORT 跟踪（每帧无条件更新一次，供违规判定与 ADAS 感知共用）=====
    ++impl_->frame_count;

    std::vector<float> pboxes;
    for (auto &d : scene_detections) {
      if (d.class_id == 1) {
        pboxes.insert(pboxes.end(), {(float)d.left, (float)d.top, (float)d.right, (float)d.bottom});
      }
    }
    int np = (int)pboxes.size() / 4;
    auto pt = impl_->person_tracker.update(pboxes.data(), np);

    std::vector<float> vboxes;
    for (auto &d : scene_detections) {
      if (d.class_id == 10) {
        vboxes.insert(vboxes.end(), {(float)d.left, (float)d.top, (float)d.right, (float)d.bottom});
      }
    }
    int nv = (int)vboxes.size() / 4;
    auto vt = impl_->vehicle_tracker.update(vboxes.data(), nv);

    // 缓存最近一帧（build_adas_perception 用）
    impl_->last_person_tracks = pt;
    impl_->last_vehicle_tracks = vt;
    impl_->last_analysis = a;
    impl_->last_detections = scene_detections;
    impl_->last_frame_w = width;
    impl_->last_frame_h = height;

    // D: 行人横穿马路（判定条件与原实现一致：有斑马线且非通行/闯红灯状态）
    if (a.has_zebra && !a.person_on_zebra && !a.person_on_zebra_red) {
      for (auto &t : pt) {
        int o = 0;
        float px = t.cx, py = t.cy;
        bool inz = (px >= a.zebra_detection.left && px <= a.zebra_detection.right &&
                    py >= a.zebra_detection.top && py <= a.zebra_detection.bottom);
        if (!inz) ++o;
        if (o > 0) {
          frame_result.alert_marker.visible = true;
          frame_result.alert_marker.center_x = (int)t.cx;
          frame_result.alert_marker.center_y = (int)t.cy;
          frame_result.alert_marker.fill_r = 255; frame_result.alert_marker.fill_g = 165; frame_result.alert_marker.fill_b = 0;
          frame_result.alert_marker.radius = 16;
          frame_result.alert_marker.text = "J";
          frame_result.status_text = "WARNING: Pedestrian jaywalking!";
          break;
        }
      }
    }
    // E: 车辆逆行
    {
      // Learn lane direction via EMA
      for (auto &t : vt) {
        if (std::abs(t.vx) < 0.5f && std::abs(t.vy) < 0.5f) continue;
        float m = std::sqrt(t.vx*t.vx + t.vy*t.vy);
        if (m < 2.f) continue;
        impl_->lane_vx = impl_->lane_vx * 0.95f + (t.vx/m) * 0.05f;
        impl_->lane_vy = impl_->lane_vy * 0.95f + (t.vy/m) * 0.05f;
        ++impl_->lane_samples;
      }
      // Detect wrong-way
      for (auto &t : vt) {
        if (impl_->lane_samples < 10) break;
        float m = std::sqrt(t.vx*t.vx + t.vy*t.vy);
        if (m < 2.f) continue;
        float dot = (t.vx/m)*impl_->lane_vx + (t.vy/m)*impl_->lane_vy;
        if (dot < -0.5f) {
          frame_result.alert_marker.visible = true;
          frame_result.alert_marker.center_x = (int)t.cx;
          frame_result.alert_marker.center_y = (int)t.cy;
          frame_result.alert_marker.fill_r = 255; frame_result.alert_marker.fill_g = 0; frame_result.alert_marker.fill_b = 0;
          frame_result.alert_marker.stroke_r = 255; frame_result.alert_marker.stroke_g = 255; frame_result.alert_marker.stroke_b = 0;
          frame_result.alert_marker.radius = 16;
          frame_result.alert_marker.text = "W";
          frame_result.status_text = "VIOLATION: Wrong-way vehicle detected!";
          break;
        }
      }
    }

    const auto t_rules = clock::now();

    // 计时输出：每 30 帧打印一次，避免刷屏
    {
      static int log_skip = 0;
      if (++log_skip >= 30) {
        log_skip = 0;
        using ms = std::chrono::duration<double, std::milli>;
        qDebug().nospace()
            << "[TRAFFIC] total:" << ms(t_rules - t_total_0).count()
            << "ms | pre:" << ms(t_pre - t_total_0).count()
            << "ms | infer:" << ms(t_infer - t_pre).count()
            << "ms | post:" << ms(t_post - t_infer).count()
            << "ms | map+rules:" << ms(t_rules - t_post).count()
            << "ms | detections:" << detections.results.size();
      }
    }
  } catch (const std::exception &ex) {
    frame_result.model_ready = false;
    frame_result.error_message = ex.what();
  }

  return frame_result;
}

// 组装最近一帧的 ADAS 感知快照（须在 infer_rgb565 之后调用）
adas::AdasPerception TrafficSceneDetector::build_adas_perception() const {
  adas::AdasPerception p;

  // ---- 信号灯：在灯类检测（2红/3绿/4黑/5黄）中选最高置信度 ----
  // 同帧多灯并存时按 红>黄>绿 优先（保守），黑灯视为无信号
  const TrafficSceneDetection *best_light = nullptr;
  adas::LightState best_state = adas::LightState::Unknown;
  float best_score = -1.0f;
  for (const auto &d : impl_->last_detections) {
    adas::LightState state = adas::LightState::Unknown;
    switch (d.class_id) {
      case 2: state = adas::LightState::Red; break;
      case 3: state = adas::LightState::Green; break;
      case 4: state = adas::LightState::Black; break;
      case 5: state = adas::LightState::Yellow; break;
      default: continue;
    }
    // 优先级权重：红 3.0 / 黄 2.0 / 绿 1.0 / 黑 0.5，乘以置信度
    const float priority = state == adas::LightState::Red ? 3.0f
                           : state == adas::LightState::Yellow ? 2.0f
                           : state == adas::LightState::Green ? 1.0f : 0.5f;
    const float score = priority * d.confidence;
    if (score > best_score) {
      best_score = score;
      best_light = &d;
      best_state = state;
    }
  }
  if (best_light != nullptr && best_state != adas::LightState::Black) {
    p.light = best_state;
    p.light_cx = (best_light->left + best_light->right) / 2;
    p.light_cy = (best_light->top + best_light->bottom) / 2;
  }

  // ---- 斑马线/行人标志（复用规则分析结果）----
  const TrafficSceneAnalysis &a = impl_->last_analysis;
  p.has_zebra = a.has_zebra;
  p.person_on_zebra = a.person_on_zebra;
  p.person_on_zebra_red = a.person_on_zebra_red;

  // ---- 障碍物：SORT 轨迹（行人 + 车辆）----
  auto append_tracks = [&p](const std::vector<sort::Sort::Track> &tracks,
                            int class_id) {
    for (const auto &t : tracks) {
      adas::ObstacleInfo ob;
      ob.class_id = class_id;
      ob.track_id = t.id;
      ob.cx = t.cx;
      ob.cy = t.cy;
      ob.vx = t.vx;
      ob.vy = t.vy;
      ob.left = static_cast<int>(t.x1);
      ob.top = static_cast<int>(t.y1);
      ob.right = static_cast<int>(t.x2);
      ob.bottom = static_cast<int>(t.y2);
      p.obstacles.push_back(ob);
    }
  };
  append_tracks(impl_->last_person_tracks, 1);   // 行人
  append_tracks(impl_->last_vehicle_tracks, 10); // 车辆

  p.frame_w = impl_->last_frame_w;
  p.frame_h = impl_->last_frame_h;
  p.valid = true;
  return p;
}
