#ifndef ADAS_NEURAL_CONTRACT_H
#define ADAS_NEURAL_CONTRACT_H
#include "core/adas_types.h"
#include <QString>
#include <array>
#include <cstddef>
#include <vector>

namespace adas {
enum class SceneModelRole { Traffic, Vehicle };
struct NeuralModelSpec {
  QString path, sha256, output, runtime_api_prefix;
  int width = 0, height = 0;
  std::vector<std::string> classes;
  std::vector<int> selected_classes; // Empty selects all trained classes.
  std::array<std::array<float,6>,3> anchors{};
};
struct NeuralBundleSpec {
  QString version;
  NeuralModelSpec traffic, vehicle;
  bool dual = false;
};
bool read_neural_manifest(const QString &path, NeuralBundleSpec &spec, QString &error);
struct NeuralTensorSpec {QString name;std::vector<unsigned> shape;};
bool validate_neural_outputs(const std::vector<NeuralTensorSpec> &tensors,
    size_t classes, int input_width, int input_height, std::string &error);
struct NeuralDetection {int class_id=-1;Box box;float confidence=0;};
struct SceneDecodeStats {
  size_t candidate_count = 0;
  size_t nms_kept = 0;
  size_t nms_comparisons = 0;
  size_t probability_clamps = 0;
};
bool decode_neural_scene(const std::vector<std::vector<float>> &heads,
    const NeuralModelSpec &spec, std::vector<NeuralDetection> &detections,
    std::string &error, SceneDecodeStats *stats = nullptr);

}
#endif
