#pragma once

#include <math.h>
#include <stdlib.h>

#include <set>
#include <vector>

#include "ztu/nn/structures.hpp"

namespace ztu {
namespace nn {

// YOLOv5 后处理器配置：包含每个检测层的锚点参数和步长
//
// anchors: 3 个检测层，每层 3 个锚点（宽, 高），共 18 个浮点值
// strides: 每个检测层的下采样倍率（通常为 8, 16, 32）
// num_strides: 检测层数量（通常为 3）
//
// 车牌检测锚点示例：{{10,13, 16,30, 33,23}, {30,61, 62,45, 59,119}, {116,90, 156,198, 373,326}}
// 交通场景锚点示例：{{5.64,12.39, 7.85,18.5, 10.72,28.14}, {63.5,9.08, 15.92,41.3, 25.48,69.9}, {218.4,16.3, 49.9,100.3, 612.5,90.0}}
struct YoloV5PostprocessConfig {
  float anchors[3][6];       // 3 层 × 6 个值（每层 3 个锚点 × 2 维宽高）
  int strides[3] = {8, 16, 32}; // 每层的下采样步长
  int num_strides = 3;          // 检测层数
};

// 统一 YOLOv5 后处理器：支持车牌检测和交通场景检测（通过不同锚点配置区分）
//
// 算法流程：
// 1. process_fp32 — 遍历每个 grid cell 的 3 个锚点：
//    a. sigmoid 解码 box_confidence，低于阈值则跳过
//    b. sigmoid 解码 box_x/y/w/h（YOLOv5 公式）：
//       x = (sigmoid(x_raw)*2 - 0.5 + col) * stride
//       y = (sigmoid(y_raw)*2 - 0.5 + row) * stride
//       w = (sigmoid(w_raw)*2)^2 * anchor_w
//       h = (sigmoid(h_raw)*2)^2 * anchor_h
//       x -= w/2, y -= h/2（中心点转左上角）
//    c. 取最大类别概率，低于阈值则跳过
//    d. 记录 box_xywh + obj_prob + class_id
//
// 2. quick_sort_indice_inverse — 按 obj_prob 降序排列所有候选框
//
// 3. NMS — 按类别分组，对每类执行非极大值抑制：
//    按置信度降序遍历，移除 IoU > threshold 的重叠框
//
// 4. 构建结果 — 过滤标记为 -1 的框，clamp 坐标到图像边界
class YoloV5Postprocessor {
public:
  explicit YoloV5Postprocessor(const YoloV5PostprocessConfig &config)
      : config_(config) {}

  // 执行后处理
  // outputs: 3 个检测头的原始输出数组指针
  // conf_threshold: 置信度阈值（低于此值的候选框被丢弃）
  // nms_threshold: NMS IoU 阈值（高于此值的重叠框被合并）
  // od_results: 输出结果列表
  // model_in_w/h: 模型输入尺寸（通常 640）
  // num_classes: 类别数量
  int postprocess(const float *outputs[], float conf_threshold,
                  float nms_threshold, object_detect_result_list *od_results,
                  int model_in_w = 640, int model_in_h = 640,
                  int num_classes = 80) {
    const int prop_box_size = 5 + num_classes; // 每个候选框的属性数：x+y+w+h+obj_conf+class_scores
    std::vector<float> filter_boxes;            // 通过置信度过滤的检测框 [x,y,w,h]*N
    std::vector<float> obj_probs;               // 对应的目标概率
    std::vector<int> class_ids;                 // 对应的类别 ID
    int valid_count = 0;
    od_results->results.clear();

    // 计算每层的 grid 尺寸
    std::vector<int> grid_hs(config_.num_strides);
    std::vector<int> grid_ws(config_.num_strides);
    for (int i = 0; i < config_.num_strides; ++i) {
      grid_hs[i] = model_in_h / config_.strides[i];
      grid_ws[i] = model_in_w / config_.strides[i];
    }

    // 步骤 1：遍历所有检测层，提取置信度超过阈值的候选框
    for (int i = 0; i < config_.num_strides; ++i) {
      valid_count += process_fp32(const_cast<float *>(outputs[i]),
                                  config_.anchors[i], grid_hs[i], grid_ws[i],
                                  config_.strides[i],
                                  filter_boxes, obj_probs, class_ids,
                                  conf_threshold, prop_box_size, num_classes);
    }

    if (valid_count <= 0) {
      return 0;
    }

    // 步骤 2：按目标概率降序排序
    std::vector<int> index_array;
    index_array.reserve(valid_count);
    for (int i = 0; i < valid_count; ++i) {
      index_array.push_back(i);
    }
    quick_sort_indice_inverse(obj_probs, 0, valid_count - 1, index_array);

    // 步骤 3：按类别分别执行 NMS
    std::set<int> class_set(class_ids.begin(), class_ids.end());
    for (int class_id : class_set) {
      nms(valid_count, filter_boxes, class_ids, index_array, class_id,
          nms_threshold);
    }

    // 步骤 4：构建输出结果
    od_results->results.reserve(valid_count);
    for (int i = 0; i < valid_count; ++i) {
      if (index_array[i] == -1) {
        continue; // NMS 中被抑制的框
      }

      const int index = index_array[i];
      const float x1 = filter_boxes[index * 4 + 0];
      const float y1 = filter_boxes[index * 4 + 1];
      const float x2 = x1 + filter_boxes[index * 4 + 2];
      const float y2 = y1 + filter_boxes[index * 4 + 3];

      object_detect_result result;
      result.box.left = clamp(x1, 0, model_in_w);
      result.box.top = clamp(y1, 0, model_in_h);
      result.box.right = clamp(x2, 0, model_in_w);
      result.box.bottom = clamp(y2, 0, model_in_h);
      result.prop = obj_probs[i];         // 使用排序后的位置 i（obj_probs 已排序）
      result.cls_id = class_ids[index];   // class_ids 未排序，需使用原始索引
      od_results->results.push_back(result);
    }

    od_results->count = od_results->results.size();
    return 0;
  }

private:
  YoloV5PostprocessConfig config_;

  // 整数值裁剪到 [min, max] 范围
  static int clamp(float val, int min, int max) {
    return val > min ? (val < max ? val : max) : min;
  }

  // Sigmoid 激活函数：将任意实数映射到 (0, 1) 区间
  static float sigmoid(float x) { return 1.0f / (1.0f + expf(-x)); }

  // 计算两个矩形框的 IoU（交并比）：交集面积 / 并集面积
  // 返回值范围 [0, 1]，用于 NMS 判断框的重叠程度
  static float calculate_overlap(float xmin0, float ymin0, float xmax0,
                                 float ymax0, float xmin1, float ymin1,
                                 float xmax1, float ymax1) {
    const float w =
        fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0f);
    const float h =
        fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0f);
    const float i = w * h; // 交集面积
    const float u = (xmax0 - xmin0 + 1.0f) * (ymax0 - ymin0 + 1.0f) +
                    (xmax1 - xmin1 + 1.0f) * (ymax1 - ymin1 + 1.0f) - i; // 并集面积
    return u <= 0.f ? 0.f : (i / u);
  }

  // 非极大值抑制（NMS）：对同一类别的检测框，按置信度降序遍历，
  // 移除与高置信度框 IoU 超过阈值的重叠框（标记 order[j] = -1）
  static int nms(int valid_count, std::vector<float> &output_locations,
                 const std::vector<int> &class_ids, std::vector<int> &order,
                 int filter_id, float threshold) {
    for (int i = 0; i < valid_count; ++i) {
      const int n = order[i];
      if (n == -1 || class_ids[n] != filter_id) {
        continue;
      }
      for (int j = i + 1; j < valid_count; ++j) {
        const int m = order[j];
        if (m == -1 || class_ids[m] != filter_id) {
          continue;
        }

        // 构建两个框的坐标（左上角 + 右下角）
        const float xmin0 = output_locations[n * 4 + 0];
        const float ymin0 = output_locations[n * 4 + 1];
        const float xmax0 =
            output_locations[n * 4 + 0] + output_locations[n * 4 + 2];
        const float ymax0 =
            output_locations[n * 4 + 1] + output_locations[n * 4 + 3];

        const float xmin1 = output_locations[m * 4 + 0];
        const float ymin1 = output_locations[m * 4 + 1];
        const float xmax1 =
            output_locations[m * 4 + 0] + output_locations[m * 4 + 2];
        const float ymax1 =
            output_locations[m * 4 + 1] + output_locations[m * 4 + 3];

        if (calculate_overlap(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1, xmax1,
                              ymax1) > threshold) {
          order[j] = -1; // 标记为被抑制
        }
      }
    }
    return 0;
  }

  // 快速排序（降序）：同时排序 input 值和对应的 indices 索引数组
  // 排序后 obj_probs 按降序排列，index_array 随之调整
  static int quick_sort_indice_inverse(std::vector<float> &input, int left,
                                       int right, std::vector<int> &indices) {
    float key;
    int key_index;
    int low = left;
    int high = right;

    if (left < right) {
      key_index = indices[left];
      key = input[left];
      while (low < high) {
        while (low < high && input[high] <= key) {
          --high;
        }
        input[low] = input[high];
        indices[low] = indices[high];
        while (low < high && input[low] >= key) {
          ++low;
        }
        input[high] = input[low];
        indices[high] = indices[low];
      }
      input[low] = key;
      indices[low] = key_index;
      quick_sort_indice_inverse(input, left, low - 1, indices);
      quick_sort_indice_inverse(input, low + 1, right, indices);
    }

    return low;
  }

  // 处理单个检测层的输出：遍历所有 grid cell 和锚点，提取通过阈值的候选框
  //
  // 输入数据布局（CHW → HWC 转换后）：
  // input[(prop_size * anchor + prop_offset) * grid_len + row * grid_w + col]
  // prop_size = 5 + class_count（x, y, w, h, obj_conf, class_0...class_N）
  //
  // YOLOv5 框解码公式：
  // x = (sigmoid(raw_x) * 2 - 0.5 + col) * stride
  // y = (sigmoid(raw_y) * 2 - 0.5 + row) * stride
  // w = pow(sigmoid(raw_w) * 2, 2) * anchor_w
  // h = pow(sigmoid(raw_h) * 2, 2) * anchor_h
  // x -= w / 2,  y -= h / 2  → 转为左上角坐标
  static int process_fp32(float *input, const float *anchor_values, int grid_h,
                          int grid_w, int stride,
                          std::vector<float> &boxes,
                          std::vector<float> &obj_probs,
                          std::vector<int> &class_ids, float threshold,
                          int prop_box_size, int class_count) {
    int valid_count = 0;
    const int grid_len = grid_h * grid_w;

    for (int anchor_index = 0; anchor_index < 3; ++anchor_index) {
      for (int row = 0; row < grid_h; ++row) {
        for (int col = 0; col < grid_w; ++col) {
          // 读取并解码 box_confidence（sigmoid 变换）
          const float box_confidence = sigmoid(
              input[(prop_box_size * anchor_index + 4) * grid_len +
                    row * grid_w + col]);
          if (box_confidence < threshold) {
            continue;
          }

          // 计算当前 grid cell + anchor 的数据起始偏移
          const int offset =
              (prop_box_size * anchor_index) * grid_len + row * grid_w + col;
          float *in_ptr = input + offset;

          // YOLOv5 框解码（sigmoid + 缩放）
          float box_x = sigmoid(*in_ptr) * 2.0f - 0.5f;
          float box_y = sigmoid(in_ptr[grid_len]) * 2.0f - 0.5f;
          float box_w = sigmoid(in_ptr[2 * grid_len]) * 2.0f;
          float box_h = sigmoid(in_ptr[3 * grid_len]) * 2.0f;

          // 映射回原图坐标
          box_x = (box_x + col) * static_cast<float>(stride);
          box_y = (box_y + row) * static_cast<float>(stride);
          box_w = box_w * box_w * anchor_values[anchor_index * 2];
          box_h = box_h * box_h * anchor_values[anchor_index * 2 + 1];
          box_x -= box_w / 2.0f;
          box_y -= box_h / 2.0f;

          // 取最大类别概率和对应的类别索引
          float max_class_prob = sigmoid(in_ptr[5 * grid_len]);
          int max_class_id = 0;
          for (int class_index = 1; class_index < class_count; ++class_index) {
            const float prob = sigmoid(in_ptr[(5 + class_index) * grid_len]);
            if (prob > max_class_prob) {
              max_class_prob = prob;
              max_class_id = class_index;
            }
          }

          if (max_class_prob <= threshold) {
            continue;
          }

          // 记录通过阈值的候选框：目标概率 = 最大类别概率 × box_confidence
          obj_probs.push_back(max_class_prob * box_confidence);
          class_ids.push_back(max_class_id);
          boxes.push_back(box_x);
          boxes.push_back(box_y);
          boxes.push_back(box_w);
          boxes.push_back(box_h);
          ++valid_count;
        }
      }
    }

    return valid_count;
  }
};

} // namespace nn
} // namespace ztu
