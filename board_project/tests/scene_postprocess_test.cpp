#include "inference/neural_contract.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void expect(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

struct Fixture {
  adas::NeuralModelSpec spec;
  std::vector<std::vector<float>> heads;
  std::vector<adas::NeuralDetection> out;
  std::string error;
  adas::SceneDecodeStats stats;

  explicit Fixture(int size = 640) {
    spec.width = spec.height = size;
    spec.output = "yolov5_rk_sigmoid_nchw_v1";
    spec.classes = {"crosswalk", "person", "light_red", "light_green", "light_black", "light_yellow"};
    for (auto &anchors : spec.anchors) anchors = {2, 2, 2, 2, 2, 2};
    for (int stride : {8, 16, 32}) heads.emplace_back(3 * 11 * (size / stride) * (size / stride), 0.f);
  }

  void candidate(int head, int anchor, int cell, int cls = 1, float confidence = .9f) {
    const int side = spec.width / (8 << head), grid = side * side;
    const int base = anchor * 11 * grid + cell;
    for (int property = 0; property < 4; ++property) heads[head][base + property * grid] = .5f;
    heads[head][base + 4 * grid] = confidence;
    heads[head][base + (5 + cls) * grid] = 1.f;
  }

  void dense(int count) {
    for (auto &anchors : spec.anchors) anchors.fill(1280.f);
    for (int h = 0; h < 3 && count; ++h) {
      const int side = spec.width / (8 << h), grid = side * side;
      for (int anchor = 0; anchor < 3 && count; ++anchor) {
        for (int cell = 0; cell < grid && count; ++cell, --count) {
          candidate(h, anchor, cell);
          const int base = anchor * 11 * grid + cell;
          heads[h][base + 2 * grid] = heads[h][base + 3 * grid] = 1.f;
        }
      }
    }
    expect(count == 0, "dense fixture exceeds the tensor shape");
  }

  bool decode() { return adas::decode_neural_scene(heads, spec, out, error, &stats); }
};

void dense_candidates_are_suppressed_before_the_output_limit() {
  Fixture f;
  f.dense(1025);
  expect(f.decode(), "1025 overlapping candidates must reach NMS");
  expect(f.out.size() == 1, "overlapping candidates must reduce to one detection");
  expect(f.stats.candidate_count == 1025 && f.stats.nms_kept == 1, "diagnostics must distinguish candidates from NMS survivors");
  expect(f.stats.nms_comparisons == 1024, "each duplicate must be compared once");
}

void the_full_tensor_candidate_bound_is_supported() {
  Fixture f;
  const int count = 3 * (80 * 80 + 40 * 40 + 20 * 20);
  f.dense(count);
  expect(f.decode(), "all candidates in the current 640 tensor must reach NMS");
  expect(f.out.size() == 1, "full dense heads must suppress to one result");
  expect(f.stats.candidate_count == size_t(count), "diagnostics must count every tensor candidate");
  expect(f.stats.nms_comparisons == size_t(count - 1), "dense natural-bound work must remain linear");
}

void legacy_960_tensors_are_rejected() {
  Fixture f(960);
  f.candidate(0, 0, 0);
  f.out.push_back({});
  f.stats = {99, 99, 99, 99};
  expect(!f.decode() && f.out.empty(), "960 input tensors must be rejected and clear detections");
  expect(f.stats.candidate_count == 0 && f.stats.nms_kept == 0 && f.stats.nms_comparisons == 0 && f.stats.probability_clamps == 0,
      "unsupported dimensions must fail before candidate processing and clear diagnostics");
}

void actual_output_overflow_fails_and_clears_results() {
  Fixture f;
  for (int cell = 0; cell < 300; ++cell) f.candidate(0, 0, cell);
  expect(f.decode() && f.out.size() == 300, "300 separated boxes must remain valid");
  f.candidate(0, 0, 300);
  expect(!f.decode(), "301 separated boxes must fail");
  expect(f.error == "scene output limit exceeded", "overflow must identify the NMS output limit");
  expect(f.out.empty(), "output overflow must discard every detection");
  expect(f.stats.candidate_count == 301 && f.stats.nms_kept == 301, "failed saturation must report the 301st survivor");
  expect(f.stats.nms_comparisons == 45150, "NMS must stop when the 301st survivor proves saturation");
}

void saturation_ignores_lower_ranked_candidates() {
  Fixture f;
  for (int cell = 0; cell < 301; ++cell) f.candidate(0, 0, cell);
  for (int cell = 0; cell < 5000; ++cell) f.candidate(0, 1, cell, 1, .3f);
  expect(!f.decode() && f.out.empty(), "301 separated high-score boxes must still fail");
  expect(f.error == "scene output limit exceeded", "saturation must preserve the failure reason");
  expect(f.stats.candidate_count == 5301 && f.stats.nms_kept == 301, "saturation diagnostics must count the complete candidate population");
  expect(f.stats.nms_comparisons == 45150, "proof of saturation must not compare lower-ranked tail candidates");
}

void all_heads_are_validated_before_candidate_processing() {
  Fixture f;
  f.dense(1025);
  f.heads[2].pop_back();
  expect(!f.decode() && f.out.empty(), "a malformed later head must fail");
  expect(f.error == "scene tensor shape/values invalid", "tensor validation must precede candidate handling");
  expect(f.stats.candidate_count == 0 && f.stats.nms_kept == 0 && f.stats.nms_comparisons == 0,
      "malformed tensors must not enter candidate processing");
}

void small_probability_roundoff_is_clamped() {
  Fixture f;
  f.candidate(0, 0, 0);
  const int grid = 80 * 80;
  const float epsilon = std::numeric_limits<float>::epsilon();
  f.heads[2][0] = -epsilon / 2;
  f.heads[0][0] = -epsilon;
  f.heads[0][2 * grid] = 1.f + epsilon;
  f.heads[0][6 * grid] = 1.f + epsilon;
  expect(f.decode() && f.out.size() == 1, "floating point sigmoid roundoff must be accepted");
  expect(std::abs(f.out[0].box.left + 8.f) < 1e-6f, "negative xy activation must clamp to zero");
  expect(std::abs(f.out[0].box.right) < 1e-6f, "overshooting width activation must clamp to one");
  expect(std::abs(f.out[0].confidence - .9f) < 1e-6f, "overshooting class activation must clamp to one");
  expect(f.stats.probability_clamps == 4, "diagnostics must include roundoff in inactive positions");
}

void diagnostics_are_reset_on_every_call() {
  Fixture f;
  f.dense(1025);
  expect(f.decode(), "setup decode must succeed");
  f.spec.width = 0;
  expect(!f.decode() && f.out.empty(), "invalid dimensions must fail");
  expect(f.stats.candidate_count == 0 && f.stats.nms_kept == 0 && f.stats.nms_comparisons == 0 && f.stats.probability_clamps == 0,
      "failure at the first guard must not retain previous diagnostics");
  f = Fixture();
  f.stats = {99, 99, 99, 99};
  expect(f.decode() && f.out.empty(), "empty scene must remain valid");
  expect(f.stats.candidate_count == 0 && f.stats.nms_kept == 0 && f.stats.nms_comparisons == 0 && f.stats.probability_clamps == 0,
      "empty scenes must clear diagnostics");
}

void material_probability_errors_and_nonfinite_values_fail() {
  for (const float value : {-8 * std::numeric_limits<float>::epsilon(),
                           1.f + 8 * std::numeric_limits<float>::epsilon(),
                           std::numeric_limits<float>::quiet_NaN(),
                           std::numeric_limits<float>::infinity()}) {
    Fixture f;
    f.candidate(0, 0, 0);
    f.heads[2][0] = value;
    f.out.push_back({});
    expect(!f.decode() && f.out.empty(), "material and nonfinite probability errors must fail and clear outputs");
  }
}

void equal_scores_preserve_tensor_traversal_order() {
  Fixture f;
  for (int cell = 0; cell < 64; ++cell) f.candidate(0, 0, cell);
  expect(f.decode() && f.out.size() == 64, "separated score ties must remain distinct");
  for (int cell = 0; cell < 64; ++cell)
    expect(f.out[cell].box.left == cell * 8.f + 3.f, "tied candidates must retain tensor traversal order");
}

void threshold_and_classwise_nms_are_preserved() {
  Fixture f;
  f.candidate(0, 0, 0, 1, .25f);
  f.candidate(0, 1, 0, 1, .9f);
  f.candidate(0, 2, 0, 0, .8f);
  expect(f.decode() && f.out.size() == 2, "NMS must suppress only same-class duplicates");
  expect(f.out[0].class_id == 1 && f.out[1].class_id == 0, "class and confidence ordering must be preserved");
  f = Fixture();
  f.candidate(0, 0, 0, 1, .25f);
  expect(f.decode() && f.out.size() == 1, "score equal to .25 remains accepted");
  f.heads[0][4 * 80 * 80] = std::nextafter(.25f, 0.f);
  expect(f.decode() && f.out.empty(), "score below .25 remains rejected");
}
}

int main() {
  struct Test { const char *name; void (*run)(); };
  const Test tests[] = {
    {"dense overlap", dense_candidates_are_suppressed_before_the_output_limit},
    {"natural tensor bound", the_full_tensor_candidate_bound_is_supported},
    {"reject legacy 960 tensors", legacy_960_tensors_are_rejected},
    {"true output overflow", actual_output_overflow_fails_and_clears_results},
    {"ignore low-ranked tail after saturation", saturation_ignores_lower_ranked_candidates},
    {"complete tensor validation", all_heads_are_validated_before_candidate_processing},
    {"small sigmoid roundoff", small_probability_roundoff_is_clamped},
    {"invalid probabilities", material_probability_errors_and_nonfinite_values_fail},
    {"diagnostics reset", diagnostics_are_reset_on_every_call},
    {"deterministic score ties", equal_scores_preserve_tensor_traversal_order},
    {"classwise NMS and threshold", threshold_and_classwise_nms_are_preserved},
  };
  int failures = 0;
  for (const auto &test : tests) {
    try { test.run(); std::cout << "PASS " << test.name << '\n'; }
    catch (const std::exception &e) { ++failures; std::cerr << "FAIL " << test.name << ": " << e.what() << '\n'; }
  }
  return failures ? 1 : 0;
}
