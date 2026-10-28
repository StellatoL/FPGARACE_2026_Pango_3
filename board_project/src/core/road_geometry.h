#ifndef ADAS_ROAD_GEOMETRY_H
#define ADAS_ROAD_GEOMETRY_H

#include "core/adas_types.h"
#include <opencv2/core.hpp>

namespace adas {
class RoadGeometry {
public:
  GeometryObservation analyze_lane(const cv::Mat &bgr, const Stamp &stamp);
  void reset();
};
} // namespace adas
#endif
