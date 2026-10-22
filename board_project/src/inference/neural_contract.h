#ifndef ADAS_NEURAL_CONTRACT_H
#define ADAS_NEURAL_CONTRACT_H
#include "core/adas_types.h"
#include <QString>
#include <array>
#include <vector>

namespace adas {
struct NeuralModelSpec {
  QString path, sha256, output, runtime_api_prefix;
  int width = 0, height = 0;
  std::vector<std::string> classes;
  std::array<std::array<float,6>,3> anchors{};
};
struct NeuralBundleSpec {
  QString version;
  NeuralModelSpec scene, lane;
  Roi road_roi{0,.42,1,.58};
};
bool read_neural_manifest(const QString &path, NeuralBundleSpec &spec, QString &error);
struct NeuralTensorSpec {QString name;std::vector<unsigned> shape;};
bool validate_neural_outputs(const std::vector<NeuralTensorSpec> &tensors,
    bool scene, size_t classes, std::string &error);
struct NeuralDetection {int class_id=-1;Box box;float confidence=0;};
bool decode_neural_scene(const std::vector<std::vector<float>> &heads,
    const NeuralModelSpec &spec, std::vector<NeuralDetection> &detections,
    std::string &error);
struct LaneTensorData {
  static constexpr int Candidates = 4, Rows = 96;
  std::vector<float> x, present, paint, type;
};
// Structured network outputs only; no input pixels are inspected.
bool decode_neural_lane(const LaneTensorData &data, const Roi &roi,
    int width, int height, LaneInfo &lane, std::string &error);
}
#endif
