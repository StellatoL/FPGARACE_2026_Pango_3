#ifndef ADAS_NEURAL_MODELS_H
#define ADAS_NEURAL_MODELS_H
#include "common/frame_packet.h"
#include "common/vision_result.h"
#include "inference/neural_contract.h"
#include <memory>
namespace adas {
struct NeuralSceneResult {VisionFrameResult vision;GeometryObservation navigation;};
// All calls belong to one NPU executor. No DMA ownership or FPGA API here.
class NeuralModels {
public:
  NeuralModels();~NeuralModels();
  bool initialize(const QString &root);
  bool lane_configured() const;
  NeuralSceneResult scene(const FramePacket &packet,ViewId view,
      const std::array<Roi,3> &light_rois,const Roi &nav_roi,const Roi &crop={0,0,1,1});
  GeometryObservation lane(const FramePacket &packet);
  QString description() const;
private:
  struct State;std::unique_ptr<State> state_;
};
}
#endif
