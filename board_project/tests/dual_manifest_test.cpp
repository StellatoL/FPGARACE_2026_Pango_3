#include "inference/neural_contract.h"
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void expect(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

const std::vector<std::string> traffic_names{
    "crosswalk", "person", "light_red", "light_green", "light_black", "light_yellow"};
const std::vector<std::string> combined_names{
    "person", "vehicle", "crosswalk", "light_red", "light_green", "cone", "barrier", "debris"};
const std::vector<std::string> coco_names{
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
    "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
    "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet",
    "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"};
const std::array<std::array<float,6>,3> traffic_anchors{{
    {5.64062f,12.39062f,7.85547f,18.5f,10.71875f,28.14062f},
    {63.5f,9.07812f,15.92188f,41.3125f,25.48438f,69.9375f},
    {218.375f,16.32812f,49.90625f,100.3125f,612.5f,90.f}}};

QJsonArray names_json(const std::vector<std::string> &names) {
  QJsonArray result;
  for (const auto &name : names) result.append(QString::fromStdString(name));
  return result;
}

struct Fixture {
  QTemporaryDir directory;
  QJsonObject manifest;
  adas::NeuralBundleSpec bundle;
  QString error;

  Fixture() {
    expect(directory.isValid(), "temporary directory must be created");
    const auto traffic = model("traffic.rknn", traffic_names, "yolov5_raw_nchw_v1");
    const auto vehicle = model("vehicle.rknn", coco_names, "yolov5_rk_sigmoid_nchw_v1");
    manifest = {{"schema", 3}, {"target", "rk3568"}, {"version", "dual640-test"},
        {"toolkit_version", "2.3.2"}, {"traffic", traffic}, {"vehicle", vehicle}};
  }

  QJsonObject model(const char *filename, const std::vector<std::string> &names, const char *output) {
    const QByteArray bytes = QByteArray("manifest test fixture, not an RKNN model: ") + filename;
    QFile file(directory.filePath(filename));
    expect(file.open(QIODevice::WriteOnly), "fixture model must be writable");
    expect(file.write(bytes) == bytes.size(), "fixture model bytes must be written");
    const auto hash = QString::fromLatin1(QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
    QJsonArray anchors;
    const auto dimensions = names == traffic_names ? traffic_anchors :
        std::array<std::array<float,6>,3>{{{10,13,16,30,33,23}, {30,61,62,45,59,119}, {116,90,156,198,373,326}}};
    for (const auto &head : dimensions) {
      QJsonArray values;
      for (const float value : head) values.append(double(value));
      anchors.append(values);
    }
    return {{"file", filename}, {"sha256", hash}, {"quantization", "int8"}, {"runtime_api_prefix", "2.3.2 ("},
        {"input", QJsonObject{{"width", 640}, {"height", 640}, {"layout", "NHWC"}, {"format", "RGB8"},
            {"mean", QJsonArray{0, 0, 0}}, {"std", QJsonArray{255, 255, 255}}}},
        {"output", output}, {"classes", names_json(names)},
        {"anchors", anchors}};
  }

  bool read() {
    QFile file(directory.filePath("manifest.json"));
    expect(file.open(QIODevice::WriteOnly | QIODevice::Truncate), "manifest must be writable");
    file.write(QJsonDocument(manifest).toJson());
    file.close();
    return adas::read_neural_manifest(file.fileName(), bundle, error);
  }
};

void schema3_loads_distinct_roles() {
  Fixture f;
  expect(f.read(), "schema3 traffic6 + COCO80 manifest must load");
  expect(f.error.isEmpty(), "valid schema3 must not report an error");
  expect(f.bundle.dual && f.bundle.traffic.classes == traffic_names && f.bundle.vehicle.classes == coco_names,
      "schema3 must expose traffic and vehicle in independent role fields");
  expect(f.bundle.traffic.anchors == traffic_anchors, "traffic6 must retain its original training anchors");
  expect(f.bundle.vehicle.selected_classes == std::vector<int>({0,1,2,3,5,7}),
      "schema3 COCO vehicle role must select only person and the configured road vehicle classes");
  expect(f.bundle.traffic.selected_classes.empty(), "traffic role must retain every trained class");
}

void schema3_requires_both_model_roles() {
  for (const char *role : {"traffic", "vehicle"}) {
    Fixture f;
    f.manifest.remove(role);
    expect(!f.read(), "a missing schema3 model role must fail");
    f.manifest[role] = QJsonValue(QJsonValue::Null);
    expect(!f.read(), "a null schema3 model role must fail");
  }
}

void schema3_requires_fixed_640_rgb_inputs() {
  for (const char *role : {"traffic", "vehicle"}) {
    for (const char *field : {"width", "height", "format", "layout", "mean", "std"}) {
      Fixture f;
      auto model = f.manifest[role].toObject();
      auto input = model["input"].toObject();
      if (QString(field) == "width" || QString(field) == "height") input[field] = 960;
      else if (QString(field) == "format") input[field] = "BGR8";
      else if (QString(field) == "layout") input[field] = "NCHW";
      else input[field] = QJsonArray{1, 1, 1};
      model["input"] = input;
      f.manifest[role] = model;
      expect(!f.read(), "schema3 must reject dimension, format, layout or normalization mismatch");
    }
  }
  Fixture f;
  auto model = f.manifest["traffic"].toObject();
  auto input = model["input"].toObject();
  input["width"] = input["height"] = 960;
  model["input"] = input;
  f.manifest["traffic"] = model;
  expect(!f.read(), "schema3 must reject square 960 models");
}

void schema3_requires_exact_class_order() {
  for (const char *role : {"traffic", "vehicle"}) {
    Fixture f;
    auto model = f.manifest[role].toObject();
    auto names = model["classes"].toArray();
    const QJsonValue first = names[0];
    names[0] = names[1]; names[1] = first;
    model["classes"] = names;
    f.manifest[role] = model;
    expect(!f.read(), "schema3 class order must match each trained model");
    names.removeLast();
    model["classes"] = names;
    f.manifest[role] = model;
    expect(!f.read(), "schema3 must reject missing classes");
  }
}

void schema3_checks_hashes_and_independent_safe_paths() {
  for (const char *role : {"traffic", "vehicle"}) {
    Fixture f;
    auto model = f.manifest[role].toObject();
    model["sha256"] = QString(64, '0');
    f.manifest[role] = model;
    expect(!f.read(), "each schema3 model hash must be verified");
    model["file"] = "../outside.rknn";
    f.manifest[role] = model;
    expect(!f.read(), "schema3 model paths must stay inside the bundle");
  }
  Fixture f;
  auto vehicle = f.manifest["vehicle"].toObject();
  const auto traffic = f.manifest["traffic"].toObject();
  vehicle["file"] = traffic["file"];
  vehicle["sha256"] = traffic["sha256"];
  f.manifest["vehicle"] = vehicle;
  expect(!f.read(), "traffic and vehicle roles must reference separate model files");

  Fixture alias;
  expect(QFile::link(alias.directory.filePath("traffic.rknn"), alias.directory.filePath("traffic-alias.rknn")), "model alias must be created");
  vehicle = alias.manifest["vehicle"].toObject();
  vehicle["file"] = "traffic-alias.rknn";
  vehicle["sha256"] = alias.manifest["traffic"].toObject()["sha256"];
  alias.manifest["vehicle"] = vehicle;
  expect(!alias.read(), "role model independence must compare canonical paths");
}

void schema3_uses_role_metadata_instead_of_filenames() {
  Fixture f;
  f.manifest["traffic"] = f.model("coco80-960-name-only.rknn", traffic_names, "yolov5_raw_nchw_v1");
  f.manifest["vehicle"] = f.model("traffic6-960-name-only.rknn", coco_names, "yolov5_rk_sigmoid_nchw_v1");
  expect(f.read(), "model dimensions and roles must be determined from the contract rather than filenames");
  auto traffic = f.manifest["traffic"].toObject();
  auto input = traffic["input"].toObject();
  input["width"] = input["height"] = 960;
  traffic["input"] = input;
  f.manifest["traffic"] = traffic;
  expect(!f.read(), "a misleading model filename must never bypass the declared input contract");
  expect(!f.bundle.dual && f.bundle.traffic.path.isEmpty() && f.bundle.vehicle.path.isEmpty(), "a failed reread must clear previously loaded role specs");
}

void schema3_requires_int8_sdk_and_training_anchors() {
  for (const char *role : {"traffic", "vehicle"}) {
    for (const char *field : {"quantization", "runtime_api_prefix"}) {
      Fixture f;
      auto model = f.manifest[role].toObject();
      model[field] = QString(field) == "quantization" ? "fp16" : "";
      f.manifest[role] = model;
      expect(!f.read(), "each schema3 model must declare INT8 and its SDK prefix");
    }
  }
  Fixture f;
  auto traffic = f.manifest["traffic"].toObject();
  auto anchors = traffic["anchors"].toArray();
  auto head = anchors[0].toArray(); head[0] = 10;
  anchors[0] = head; traffic["anchors"] = anchors;
  f.manifest["traffic"] = traffic;
  expect(!f.read(), "traffic role must reject anchors from a different trained model");
}

void schema3_requires_role_output_contracts() {
  Fixture f;
  auto traffic = f.manifest["traffic"].toObject();
  traffic["output"] = "yolov5_rk_sigmoid_nchw_v1";
  f.manifest["traffic"] = traffic;
  expect(!f.read(), "traffic6 must use the measured raw-logit output contract");
  traffic["output"] = "yolov5_raw_nchw_v1";
  f.manifest["traffic"] = traffic;
  auto vehicle = f.manifest["vehicle"].toObject();
  vehicle["output"] = "yolov5_raw_nchw_v1";
  f.manifest["vehicle"] = vehicle;
  expect(!f.read(), "the deployed COCO80 role must reject a raw-logit output contract");
  vehicle["output"] = "flattened_xywh";
  f.manifest["vehicle"] = vehicle;
  expect(!f.read(), "COCO80 must reject unsupported output formats");
}

void schema3_rejects_configured_neural_lane_models() {
  Fixture f;
  expect(f.read(), "the current schema3 contract must work without lane metadata");
  f.manifest["lane"] = QJsonValue(QJsonValue::Null);
  expect(f.read(), "explicit null legacy lane metadata must remain harmless");
  f.manifest["lane"] = QJsonObject{};
  expect(!f.read(), "schema3 must reject any configured neural lane model");
}

void legacy_combined_manifests_are_rejected() {
  for (const int schema : {1, 2}) for (const int side : {640, 960}) {
    Fixture f;
    expect(f.read(), "a valid schema3 manifest must load before the rejected legacy reread");
    auto scene = f.model("combined.rknn", combined_names, "yolov5_rk_sigmoid_nchw_v1");
    auto input = scene["input"].toObject();
    input["width"] = input["height"] = side;
    scene["input"] = input;
    f.manifest["schema"] = schema;
    f.manifest["scene"] = scene;
    f.manifest["road_roi"] = QJsonArray{0, .42, 1, .58};
    f.manifest["lane"] = QJsonValue(QJsonValue::Null);
    f.manifest.remove("traffic"); f.manifest.remove("vehicle");
    if (schema == 1) {
      auto lane = f.model("lane.rknn", {}, "lane_rows_4x96_v1");
      input = lane["input"].toObject();
      input["width"] = 512; input["height"] = 256;
      lane["input"] = input;
      f.manifest["lane"] = lane;
    }
    expect(!f.read(), "schema1/2 combined manifests must be rejected at every legacy resolution");
    expect(!f.error.isEmpty() && !f.bundle.dual && f.bundle.version.isEmpty() &&
        f.bundle.traffic.path.isEmpty() && f.bundle.vehicle.path.isEmpty(),
        "a rejected legacy reread must report an error and clear all loaded role specs");
    scene["classes"] = names_json(traffic_names);
    f.manifest["scene"] = scene;
    expect(!f.read(), "legacy schemas must remain rejected when their class list is changed to traffic6");
  }
}

adas::NeuralModelSpec spec_for(const std::vector<std::string> &names, const char *output) {
  adas::NeuralModelSpec result;
  result.width = result.height = 640;
  result.classes = names;
  result.output = output;
  result.anchors = names == traffic_names ? traffic_anchors :
      std::array<std::array<float,6>,3>{{{10,13,16,30,33,23}, {30,61,62,45,59,119}, {116,90,156,198,373,326}}};
  return result;
}

std::vector<std::vector<float>> heads_for(size_t classes, float background) {
  std::vector<std::vector<float>> result;
  for (int stride : {8, 16, 32}) result.emplace_back(3 * (classes + 5) * (640 / stride) * (640 / stride), background);
  return result;
}

void six_class_raw_heads_keep_their_class_indices() {
  auto spec = spec_for(traffic_names, "yolov5_raw_nchw_v1");
  auto heads = heads_for(6, -20.f);
  const int grid = 20 * 20, cell = grid - 1, base = 2 * 11 * grid + cell;
  for (int p = 0; p < 4; ++p) heads[2][base + p * grid] = 0.f;
  heads[2][base + 4 * grid] = 12.f;
  heads[2][base + 9 * grid] = 12.f; // class4: light_black, rather than a permissive green state.
  std::vector<adas::NeuralDetection> out;
  std::string error;
  expect(adas::decode_neural_scene(heads, spec, out, error) && out.size() == 1, "six-class raw heads must decode");
  expect(out[0].class_id == 4 && spec.classes[out[0].class_id] == "light_black", "traffic class4 must retain black-light identity");
  expect(std::abs((out[0].box.left + out[0].box.right) / 2 - 624.f) < 1e-5f, "traffic raw heads must retain correct final-cell coordinates");
  heads[2][base + 4 * grid] = std::log(.3f / .7f);
  heads[2][base + 9 * grid] = std::log(.3f / .7f);
  expect(adas::decode_neural_scene(heads, spec, out, error) && out.empty(), "traffic raw objectness and class score must be sigmoid-activated exactly once");
}

void eighty_class_sigmoid_heads_keep_coco_indices() {
  auto spec = spec_for(coco_names, "yolov5_rk_sigmoid_nchw_v1");
  auto heads = heads_for(80, 0.f);
  const int grid = 80 * 80;
  const std::vector<int> classes{0, 2, 5, 7, 79};
  for (size_t i = 0; i < classes.size(); ++i) {
    const int cell = int(i) * 8;
    for (int p = 0; p < 4; ++p) heads[0][p * grid + cell] = .5f;
    heads[0][4 * grid + cell] = .9f - .05f * float(i);
    heads[0][(5 + classes[i]) * grid + cell] = 1.f;
  }
  std::vector<adas::NeuralDetection> out;
  std::string error;
  expect(adas::decode_neural_scene(heads, spec, out, error) && out.size() == classes.size(), "80-class sigmoid heads must decode without reshaping channels as an ADAS8 head");
  for (size_t i = 0; i < classes.size(); ++i) expect(out[i].class_id == classes[i], "COCO class indices must stay unchanged");
  expect(spec.classes[out[0].class_id] == "person" && spec.classes[out[1].class_id] == "car" && spec.classes[out[4].class_id] == "toothbrush", "COCO first, vehicle and final channels must map correctly");

  auto final_cell_heads = heads_for(80, 0.f);
  const int final_grid = 20 * 20, final_base = 2 * 85 * final_grid + final_grid - 1;
  for (int p = 0; p < 4; ++p) final_cell_heads[2][final_base + p * final_grid] = .5f;
  final_cell_heads[2][final_base + 4 * final_grid] = .9f;
  final_cell_heads[2][final_base + 83 * final_grid] = .8f; // class78, hair drier.
  expect(adas::decode_neural_scene(final_cell_heads, spec, out, error) && out.size() == 1, "COCO80 final anchor and grid cell must decode");
  expect(out[0].class_id == 78 && spec.classes[out[0].class_id] == "hair drier", "COCO class78 must retain its exact channel index");
  expect(std::abs((out[0].box.left + out[0].box.right) / 2 - 624.f) < 1e-5f &&
      std::abs((out[0].box.top + out[0].box.bottom) / 2 - 624.f) < 1e-5f, "COCO80 final grid coordinates must use the declared input width");
}

void selected_road_classes_keep_only_the_winning_coco_class() {
  auto spec = spec_for(coco_names, "yolov5_rk_sigmoid_nchw_v1");
  spec.selected_classes = {0,1,2,3,5,7};
  auto heads = heads_for(80, 0.f);
  const int grid = 80 * 80;
  for (size_t i = 0; i < spec.selected_classes.size(); ++i) {
    const int cell = int(i) * 16;
    for (int p = 0; p < 4; ++p) heads[0][p * grid + cell] = .5f;
    heads[0][4 * grid + cell] = .95f - .05f * float(i);
    heads[0][(5 + spec.selected_classes[i]) * grid + cell] = .9f;
  }
  const int animal = 400;
  for (int p = 0; p < 4; ++p) heads[0][p * grid + animal] = .5f;
  heads[0][4 * grid + animal] = 1.f;
  heads[0][(5 + 22) * grid + animal] = 1.f; // COCO zebra is an animal.
  heads[0][(5 + 2) * grid + animal] = .99f; // Never reassign it to a lesser-scoring car.
  std::vector<adas::NeuralDetection> out;
  std::string error;
  adas::SceneDecodeStats stats;
  expect(adas::decode_neural_scene(heads, spec, out, error, &stats), "selected road class heads must decode");
  expect(out.size() == spec.selected_classes.size() && stats.candidate_count == out.size(),
      "COCO animal winners must be discarded before candidate collection without reassignment");
  for (size_t i = 0; i < out.size(); ++i) {
    const int cell = int(i) * 16;
    expect(out[i].class_id == spec.selected_classes[i], "selected COCO IDs must remain unchanged");
    expect(std::abs(out[i].confidence - (.95f - .05f * float(i)) * .9f) < 1e-6f,
        "class selection must preserve objectness-times-class confidence");
    expect(std::abs((out[i].box.left + out[i].box.right) / 2 - (4.f + 8.f * (cell % 80))) < 1e-5f &&
        std::abs((out[i].box.top + out[i].box.bottom) / 2 - (4.f + 8.f * (cell / 80))) < 1e-5f &&
        std::abs(out[i].box.right - out[i].box.left - 10.f) < 1e-5f &&
        std::abs(out[i].box.bottom - out[i].box.top - 13.f) < 1e-5f,
        "class selection must preserve anchor geometry and grid coordinates");
  }
}

void excluded_class_flood_does_not_consume_the_output_limit() {
  auto spec = spec_for(coco_names, "yolov5_rk_sigmoid_nchw_v1");
  spec.selected_classes = {0,1,2,3,5,7};
  for (auto &head : spec.anchors) head.fill(2.f);
  auto heads = heads_for(80, 0.f);
  const int grid = 80 * 80;
  for (int cell = 0; cell <= 400; ++cell) {
    for (int p = 0; p < 4; ++p) heads[0][p * grid + cell] = .5f;
    heads[0][4 * grid + cell] = cell == 400 ? .8f : 1.f;
    heads[0][(5 + (cell == 400 ? 2 : 22)) * grid + cell] = 1.f;
  }
  std::vector<adas::NeuralDetection> out;
  std::string error;
  adas::SceneDecodeStats stats;
  expect(adas::decode_neural_scene(heads, spec, out, error, &stats) && out.size() == 1,
      "more than 300 excluded animal detections must leave capacity for the selected road detection");
  expect(out[0].class_id == 2 && stats.candidate_count == 1 && stats.nms_kept == 1 && stats.nms_comparisons == 0,
      "excluded classes must consume neither candidate counts nor NMS work");
  heads[2][(5 + 22) * 20 * 20] = std::numeric_limits<float>::quiet_NaN();
  expect(!adas::decode_neural_scene(heads, spec, out, error, &stats) && out.empty(),
      "class filtering must never skip finite-value validation in an excluded class or inactive head");
  expect(stats.candidate_count == 0 && stats.nms_comparisons == 0,
      "complete head validation must precede filtered candidate collection");
}

void invalid_class_filters_fail_and_clear_results() {
  auto spec = spec_for(coco_names, "yolov5_rk_sigmoid_nchw_v1");
  const auto heads = heads_for(80, 0.f);
  std::vector<adas::NeuralDetection> out;
  std::string error;
  for (const std::vector<int> filter : {std::vector<int>{-1}, std::vector<int>{80}, std::vector<int>{2,2}}) {
    spec.selected_classes = filter;
    out.push_back({});
    expect(!adas::decode_neural_scene(heads, spec, out, error) && out.empty(),
        "negative, out-of-range or duplicate selected class IDs must fail closed");
  }
}

void role_head_shapes_are_explicit() {
  std::string error;
  for (const unsigned classes : {6u, 80u}) {
    const unsigned channels = 3 * (classes + 5);
    std::vector<adas::NeuralTensorSpec> heads{{"p3", {1,channels,80,80}}, {"p4", {1,channels,40,40}}, {"p5", {1,channels,20,20}}};
    expect(adas::validate_neural_outputs(heads, classes, 640, 640, error), "six and 80-class NCHW head shapes must validate");
    heads[2].shape[1] += 3;
    expect(!adas::validate_neural_outputs(heads, classes, 640, 640, error), "head channel mismatch must fail");
  }
  for (const unsigned classes : {1u, 7u, 8u, 65u, 79u, 81u}) {
    const unsigned channels = 3 * (classes + 5);
    std::vector<adas::NeuralTensorSpec> heads{{"p3", {1,channels,80,80}}, {"p4", {1,channels,40,40}}, {"p5", {1,channels,20,20}}};
    expect(!adas::validate_neural_outputs(heads, classes, 640, 640, error), "legacy eight-class and arbitrary class counts must be rejected");
  }
}

void legacy_eight_class_heads_are_rejected() {
  const auto spec = spec_for(combined_names, "yolov5_rk_sigmoid_nchw_v1");
  const auto heads = heads_for(8, 0.f);
  std::vector<adas::NeuralDetection> out(1);
  std::string error;
  adas::SceneDecodeStats stats{99, 99, 99, 99};
  expect(!adas::decode_neural_scene(heads, spec, out, error, &stats) && out.empty() && !error.empty(),
      "the legacy eight-class decoder must reject and clear prior detections");
  expect(stats.candidate_count == 0 && stats.nms_kept == 0 && stats.nms_comparisons == 0 && stats.probability_clamps == 0,
      "unsupported classes must fail before candidate processing and clear diagnostics");
}
}

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  struct Test { const char *name; void (*run)(); };
  const Test tests[] = {
    {"schema3 role loading", schema3_loads_distinct_roles},
    {"required roles", schema3_requires_both_model_roles},
    {"640 RGB input contract", schema3_requires_fixed_640_rgb_inputs},
    {"class order", schema3_requires_exact_class_order},
    {"independent paths and hashes", schema3_checks_hashes_and_independent_safe_paths},
    {"role metadata over filenames", schema3_uses_role_metadata_instead_of_filenames},
    {"INT8 SDK and anchors", schema3_requires_int8_sdk_and_training_anchors},
    {"role output contracts", schema3_requires_role_output_contracts},
    {"reject configured neural lane", schema3_rejects_configured_neural_lane_models},
    {"reject legacy schemas", legacy_combined_manifests_are_rejected},
    {"traffic6 raw decode", six_class_raw_heads_keep_their_class_indices},
    {"COCO80 sigmoid decode", eighty_class_sigmoid_heads_keep_coco_indices},
    {"winning selected road classes", selected_road_classes_keep_only_the_winning_coco_class},
    {"excluded class flood", excluded_class_flood_does_not_consume_the_output_limit},
    {"invalid class filters", invalid_class_filters_fail_and_clear_results},
    {"role tensor shapes", role_head_shapes_are_explicit},
    {"reject legacy eight-class heads", legacy_eight_class_heads_are_rejected},
  };
  int failures = 0;
  for (const auto &test : tests) {
    try { test.run(); std::cout << "PASS " << test.name << '\n'; }
    catch (const std::exception &e) { ++failures; std::cerr << "FAIL " << test.name << ": " << e.what() << '\n'; }
  }
  return failures ? 1 : 0;
}
