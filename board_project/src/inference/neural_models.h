#ifndef ADAS_NEURAL_MODELS_H
#define ADAS_NEURAL_MODELS_H
#include "common/frame_packet.h"
#include "common/vision_result.h"
#include "inference/neural_contract.h"
#include <memory>
namespace adas {
struct NeuralInputMapping {
  bool prepared=false;
  int width=0,height=0;
  ViewRect source_crop{0,0,0,0},content{0,0,0,0};
};
struct NeuralSceneResult {
  VisionFrameResult vision;
  SceneDecodeStats decode;
  SceneModelRole role=SceneModelRole::Vehicle;
  NeuralInputMapping input_mapping;
};
// Both model contexts belong to one NPU executor; no FPGA resources here.
class NeuralModels {
public:
  NeuralModels();~NeuralModels();
  bool initialize(const QString &root);
  NeuralSceneResult scene(const FramePacket &packet,ViewId view,
      const std::array<Roi,3> &light_rois,const Roi &crop={0,0,1,1},
      SceneModelRole role=SceneModelRole::Vehicle);
  QString description() const;
private:
  struct State;std::unique_ptr<State> state_;
};
}
#endif
