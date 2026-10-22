#ifndef ADAS_ROAD_GEOMETRY_H
#define ADAS_ROAD_GEOMETRY_H

#include "core/adas_types.h"
#include "core/ground_calibration.h"
#include <opencv2/core.hpp>

namespace adas {
class RoadGeometry {
public:
  GeometryObservation analyze(const cv::Mat &bgr, const Stamp &stamp,
                              const GroundCalibration &calibration,
                              const Roi &nav_roi,
                              const std::vector<Box> &dynamic_boxes = {});
  void reset();
private:
  SpeedResult analyze_speed(const cv::Mat &gray, const Stamp &stamp,
                            const GroundCalibration &calibration,
                            const std::vector<Box> &dynamic_boxes);
  cv::Mat previous_gray_, previous_mask_;
  Stamp previous_stamp_;
  GeometryObservation previous_observation_;
  std::string calibration_key_;
  double smoothed_speed_kmh_ = -1;
  bool has_observation_ = false;
};
} // namespace adas
#endif
