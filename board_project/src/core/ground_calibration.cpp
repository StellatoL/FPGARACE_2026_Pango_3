#include "core/ground_calibration.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

namespace adas {
namespace {
bool fail(std::string *error, const char *message) {
  if (error) *error = message;
  return false;
}
bool finite(cv::Point2f p) { return std::isfinite(p.x) && std::isfinite(p.y); }
double cross(cv::Point2f a, cv::Point2f b) { return double(a.x)*b.y - double(a.y)*b.x; }
} // namespace

void GroundCalibration::reset() {
  valid_ = false;
  points_ = {};
  image_to_ground_ = cv::Matx33d::eye();
  width_m_ = length_m_ = 0;
  near_offset_m_ = -1;
  frame_w_ = frame_h_ = 0;
  vehicle_width_m_ = 1.8;
  profile_identity_.clear();
}

bool GroundCalibration::configure(const std::array<cv::Point2f,4> &points,
                                  double width, double length, double offset,
                                  int frame_w, int frame_h, std::string *error) {
  reset();
  if (error) error->clear();
  if (frame_w < 16 || frame_h < 16 || !std::isfinite(width) || !std::isfinite(length) ||
      !std::isfinite(offset) || width <= 0.05 || length <= 0.05 || width > 1000 || length > 10000 || offset > 10000)
    return fail(error, "Invalid image dimensions or metric scale");
  for (auto p : points)
    if (!finite(p) || p.x < 0 || p.y < 0 || p.x >= frame_w || p.y >= frame_h)
      return fail(error, "Calibration points must be finite native image pixels");
  for (int i=0;i<4;++i) {
    const auto edge = points[(i+1)%4]-points[i];
    const auto next = points[(i+2)%4]-points[(i+1)%4];
    if (cv::norm(edge) < 4 || cross(edge,next) < 8)
      return fail(error, "Use a convex nondegenerate quadrilateral in TL/TR/BR/BL order");
  }
  if (points[0].x >= points[1].x || points[3].x >= points[2].x ||
      points[0].y + points[1].y >= points[2].y + points[3].y ||
      std::abs(cv::contourArea(std::vector<cv::Point2f>(points.begin(),points.end()))) < 64)
    return fail(error, "Calibration edges do not have TL/TR/BR/BL image order");
  const float near_y = float(std::max(0.0,offset));
  const std::array<cv::Point2f,4> ground = {{{float(-width/2),float(near_y+length)},
      {float(width/2),float(near_y+length)},{float(width/2),near_y},{float(-width/2),near_y}}};
  try {
    const cv::Mat h = cv::getPerspectiveTransform(points.data(),ground.data());
    if (!cv::checkRange(h) || std::abs(cv::determinant(h)) < 1e-12)
      return fail(error,"Calibration homography is singular");
    image_to_ground_ = h;
    // A valid road patch cannot cross the projective horizon internally.
    double sign = 0;
    for (auto p : points) {
      const double d = image_to_ground_(2,0)*p.x + image_to_ground_(2,1)*p.y + image_to_ground_(2,2);
      if (std::abs(d) < 1e-8 || (sign != 0 && d*sign <= 0))
        return fail(error,"Calibration crosses the projective horizon");
      sign = d;
    }
  } catch (const cv::Exception &) {
    return fail(error,"Could not build calibration homography");
  }
  points_ = points;
  width_m_ = width; length_m_ = length; near_offset_m_ = offset;
  frame_w_ = frame_w; frame_h_ = frame_h;
  valid_ = true;
  return true;
}

cv::Point2f GroundCalibration::map(cv::Point2f p) const {
  const float invalid = std::numeric_limits<float>::quiet_NaN();
  if (!valid_ || !finite(p)) return {invalid,invalid};
  const cv::Vec3d q = image_to_ground_*cv::Vec3d(p.x,p.y,1);
  if (!std::isfinite(q[2]) || std::abs(q[2]) < 1e-9) return {invalid,invalid};
  const cv::Point2f mapped(float(q[0]/q[2]),float(q[1]/q[2]));
  return finite(mapped) ? mapped : cv::Point2f(invalid,invalid);
}

double GroundCalibration::distance(const Box &box) const {
  if (!distance_valid() || !std::isfinite(box.left) || !std::isfinite(box.right) ||
      !std::isfinite(box.top) || !std::isfinite(box.bottom) ||
      box.left > box.right || box.top > box.bottom) return -1;
  const cv::Point2f foot((box.left+box.right)*0.5f,box.bottom);
  if (cv::pointPolygonTest(std::vector<cv::Point2f>(points_.begin(),points_.end()),foot,false) < 0)
    return -1;
  const auto ground = map(foot);
  if (!finite(ground) || ground.y < 0 || ground.y > near_offset_m_+length_m_+0.01) return -1;
  return std::hypot(ground.x,ground.y);
}

bool GroundCalibration::is_compatible(int width, int height, const std::string &identity) const {
  return valid_ && width == frame_w_ && height == frame_h_ &&
         (identity.empty() || identity == profile_identity_);
}
bool GroundCalibration::set_vehicle_width_m(double width) {
  if (!std::isfinite(width) || width < 0.2 || width > 10) return false;
  vehicle_width_m_ = width;
  return true;
}

bool GroundCalibration::save(const std::string &path) const {
  if (!valid_) return false;
  try {
    cv::FileStorage file(path,cv::FileStorage::WRITE);
    if (!file.isOpened()) return false;
    file << "version" << 1 << "frame_width" << frame_w_ << "frame_height" << frame_h_
         << "profile_identity" << profile_identity_ << "width_m" << width_m_
         << "length_m" << length_m_ << "near_offset_m" << near_offset_m_
         << "vehicle_width_m" << vehicle_width_m_;
    file << "points" << "[";
    for (auto p : points_) file << p;
    file << "]";
    file.release();
    return true;
  } catch (const cv::Exception &) { return false; }
}

bool GroundCalibration::load(const std::string &path, int expected_w, int expected_h,
                             std::string *error) {
  reset();
  if (error) error->clear();
  // Avoid OpenCV's global error logger for the normal missing-profile case.
  if (!std::ifstream(path).good()) return fail(error,"Calibration file is unavailable");
  try {
    cv::FileStorage file(path,cv::FileStorage::READ);
    if (!file.isOpened()) return fail(error,"Cannot open calibration file");
    int version=0,w=0,h=0;
    double width=0,length=0,offset=-1,vehicle_width=1.8;
    std::string profile;
    file["version"] >> version; file["frame_width"] >> w; file["frame_height"] >> h;
    if (version != 1 || w != expected_w || h != expected_h)
      return fail(error,"Calibration version or front-view dimensions do not match");
    file["width_m"] >> width; file["length_m"] >> length;
    // OpenCV reads an absent numeric node as zero. An unmeasured origin must
    // retain the unavailable sentinel rather than authorizing guessed distances.
    if (!file["near_offset_m"].empty()) file["near_offset_m"] >> offset;
    file["profile_identity"] >> profile;
    if (!file["vehicle_width_m"].empty()) file["vehicle_width_m"] >> vehicle_width;
    const auto node = file["points"];
    if (!node.isSeq() || node.size() != 4) return fail(error,"Calibration requires four points");
    std::array<cv::Point2f,4> points;
    for (int i=0;i<4;++i) node[i] >> points[i];
    if (!configure(points,width,length,offset,w,h,error)) return false;
    if (!set_vehicle_width_m(vehicle_width)) { reset(); return fail(error,"Invalid vehicle width"); }
    profile_identity_ = profile;
    return true;
  } catch (const cv::Exception &) {
    reset(); return fail(error,"Malformed calibration file");
  }
}
} // namespace adas
