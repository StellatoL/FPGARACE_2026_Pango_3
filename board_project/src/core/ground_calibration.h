#ifndef ADAS_GROUND_CALIBRATION_H
#define ADAS_GROUND_CALIBRATION_H

#include "core/adas_types.h"
#include <opencv2/core.hpp>
#include <array>
#include <string>

namespace adas {
// Front-camera road plane only. Points are native pixels, TL/TR/BR/BL.
// Ground x is rightwards and ground y is forward, both in metres.
class GroundCalibration {
public:
  bool configure(const std::array<cv::Point2f,4> &points, double width_m,
                 double length_m, double near_offset_m, int frame_w,
                 int frame_h, std::string *error = nullptr);
  bool valid() const { return valid_; }
  bool distance_valid() const { return valid_ && near_offset_m_ >= 0; }
  cv::Point2f map(cv::Point2f pixel) const;
  double distance(const Box &box) const;
  bool save(const std::string &path) const;
  bool load(const std::string &path, int expected_w, int expected_h,
            std::string *error = nullptr);
  void reset();
  const std::array<cv::Point2f,4> &image_points() const { return points_; }
  double width_m() const { return width_m_; }
  double length_m() const { return length_m_; }
  double near_offset_m() const { return near_offset_m_; }
  int frame_width() const { return frame_w_; }
  int frame_height() const { return frame_h_; }
  void set_profile_identity(std::string identity) { profile_identity_ = std::move(identity); }
  const std::string &profile_identity() const { return profile_identity_; }
  bool is_compatible(int width, int height, const std::string &identity = {}) const;
  bool set_vehicle_width_m(double width);
  double vehicle_width_m() const { return vehicle_width_m_; }
private:
  bool valid_ = false;
  std::array<cv::Point2f,4> points_{};
  cv::Matx33d image_to_ground_ = cv::Matx33d::eye();
  double width_m_ = 0, length_m_ = 0, near_offset_m_ = -1;
  double vehicle_width_m_ = 1.8;
  int frame_w_ = 0, frame_h_ = 0;
  std::string profile_identity_;
};
} // namespace adas
#endif
