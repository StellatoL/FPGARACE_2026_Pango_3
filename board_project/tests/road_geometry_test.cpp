#include "core/ground_calibration.h"
#include "core/road_geometry.h"

#include <opencv2/imgproc.hpp>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>

namespace {
int failures = 0;
void check(bool ok, const char *name) {
  if (!ok) { ++failures; std::cerr << "FAIL: " << name << '\n'; }
}
bool close(double a, double b, double tolerance = 0.01) {
  return std::isfinite(a) && std::abs(a - b) < tolerance;
}
std::array<cv::Point2f, 4> points() {
  return {{{230, 190}, {410, 190}, {610, 460}, {30, 460}}};
}
adas::GroundCalibration calibration(double offset = 5) {
  adas::GroundCalibration c;
  check(c.configure(points(), 10, 35, offset, 640, 480), "calibration configures");
  return c;
}
adas::Stamp stamp(uint64_t seq, double time, uint64_t generation = 1) {
  adas::Stamp s; s.sequence = seq; s.source_s = time; s.generation = generation; return s;
}
void test_calibration() {
  auto c = calibration();
  check(c.valid() && c.distance_valid(), "metric calibration is valid");
  check(close(c.map(points()[0]).x, -5) && close(c.map(points()[0]).y, 40), "far left maps to metres");
  check(close(c.map(points()[2]).x, 5) && close(c.map(points()[2]).y, 5), "near right maps to metres");
  check(close(c.distance({290, 400, 350, 460}), 5), "bottom center gives near distance");
  check(c.distance({0, 0, 10, 10}) < 0, "outside calibrated plane has no distance");
  c = calibration(-1);
  check(c.valid() && !c.distance_valid() && c.distance({290, 400, 350, 460}) < 0,
        "scale without car offset supplies no obstacle distance");
  auto p = points(); std::swap(p[1], p[3]);
  check(!c.configure(p, 10, 35, 5, 640, 480) && !c.valid(), "reverse point order rejects and clears old calibration");
  p = points(); p[2] = p[1];
  check(!c.configure(p, 10, 35, 5, 640, 480), "duplicate calibration point rejects");
  p = {{{10, 10}, {20, 20}, {30, 30}, {40, 40}}};
  check(!c.configure(p, 10, 35, 5, 640, 480), "collinear calibration rejects");
  check(!c.configure(points(), 0, 35, 5, 640, 480), "zero metric width rejects");
  check(!c.configure(points(), 10, std::numeric_limits<double>::infinity(), 5, 640, 480), "infinite metric length rejects");
  c = calibration(); c.set_profile_identity("camera-A:mosaic-front");
  check(c.set_vehicle_width_m(1.9), "vehicle width configures");
  const std::string file = "/tmp/adas_ground_calibration_test.yml";
  check(c.save(file), "calibration persists");
  adas::GroundCalibration loaded;
  check(loaded.load(file, 640, 480) && loaded.profile_identity() == c.profile_identity() &&
        close(loaded.vehicle_width_m(), 1.9), "calibration reload preserves identity and vehicle width");
  check(loaded.is_compatible(640, 480, "camera-A:mosaic-front") &&
        !loaded.is_compatible(640, 480, "camera-B:mosaic-front"), "profile mismatch rejects reuse");
  check(!loaded.load(file, 1280, 720) && !loaded.valid(), "dimension mismatch clears calibration");
  loaded = calibration();
  check(!loaded.load("/tmp/adas_calibration_does_not_exist.yml", 640, 480) && !loaded.valid(), "missing file clears calibration");
  {
    cv::FileStorage scale_only(file,cv::FileStorage::WRITE);
    scale_only << "version" << 1 << "frame_width" << 640 << "frame_height" << 480
               << "width_m" << 10 << "length_m" << 35 << "points" << "[";
    for (auto point:points()) scale_only << point;
    scale_only << "]";
  }
  check(loaded.load(file,640,480) && loaded.valid() && loaded.near_offset_m()<0 &&
        !loaded.distance_valid() && loaded.distance({290,400,350,460})<0,
        "omitted near offset preserves plane scale without guessed distance");
  std::remove(file.c_str());
}
void test_lane_only_contract() {
  adas::RoadGeometry geometry;
  cv::Mat image(480,640,CV_8UC3,cv::Scalar(45,45,45));
  cv::line(image,{285,210},{139,475},cv::Scalar(255,255,255),8);
  auto source=stamp(7,1.25,3);source.receive_s=5.5;source.static_image=true;
  const auto result=geometry.analyze_lane(image,source);
  check(result.components==adas::GeometryObservation::Lane && result.status==adas::Status::Valid,
        "lane analysis publishes only a valid lane component");
  check(result.width==image.cols && result.height==image.rows &&
        result.stamp.sequence==source.sequence && result.stamp.generation==source.generation &&
        result.stamp.source_s==source.source_s && result.stamp.receive_s==source.receive_s &&
        result.stamp.static_image==source.static_image,
        "lane observation preserves native dimensions and the complete source stamp");
  check(result.lane.left_valid && result.lane.left_type==adas::LineType::Solid &&
        !result.nav.detected && !result.speed.valid && result.speed.features==0 &&
        std::isfinite(result.process_ms) && result.process_ms>=0,
        "static frames retain painted lane evidence without producing navigation or speed");
  image.setTo(cv::Scalar(45,45,45));
  const auto blank=geometry.analyze_lane(image,source);
  check(blank.status==adas::Status::Valid && !blank.lane.left_valid && !blank.lane.right_valid &&
        blank.lane.left_type==adas::LineType::Unknown && blank.lane.right_type==adas::LineType::Unknown &&
        blank.lane.left_conf==0 && blank.lane.right_conf==0,
        "a changed image with the same source stamp cannot reuse previous lane evidence");
  // Paint above the existing road ROI supplies no boundary evidence.
  cv::line(image,{320,20},{139,195},cv::Scalar(255,255,255),8);
  const auto above=geometry.analyze_lane(image,source);
  check(!above.lane.left_valid && !above.lane.right_valid,
        "markings wholly above the road ROI cannot establish a lane");
  for (const auto &invalid: {cv::Mat(),cv::Mat(480,640,CV_8UC1),cv::Mat(480,640,CV_8UC4),
                            cv::Mat(480,640,CV_32FC3),cv::Mat(480,31,CV_8UC3),cv::Mat(31,640,CV_8UC3)}) {
    const auto failed=geometry.analyze_lane(invalid,source);
    check(failed.status==adas::Status::Failed && failed.components==adas::GeometryObservation::Lane &&
          !failed.lane.left_valid && !failed.lane.right_valid &&
          failed.lane.left_type==adas::LineType::Unknown && failed.lane.right_type==adas::LineType::Unknown &&
          failed.lane.left_conf==0 && failed.lane.right_conf==0 &&
          !failed.nav.detected && !failed.speed.valid,
          "unsupported images fail without retaining lane authorization evidence");
  }
  geometry.reset();
  image.setTo(cv::Scalar(45,45,45));
  cv::line(image,{285,210},{139,475},cv::Scalar(255,255,255),8);
  const auto recovered=geometry.analyze_lane(image,stamp(1,0,4));
  check(recovered.status==adas::Status::Valid && recovered.lane.left_valid &&
        recovered.lane.left_type==result.lane.left_type &&
        recovered.lane.left_x==result.lane.left_x && recovered.lane.left_top_x==result.lane.left_top_x &&
        recovered.lane.left_conf==result.lane.left_conf,
        "reset, failed input, generation and sequence changes preserve the same spatial lane result");
}
void test_lanes() {
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(45,45,45));
  auto lx = [](int y) { return cvRound(285 - (y-210)*0.55); };
  auto rx = [](int y) { return cvRound(355 + (y-210)*0.55); };
  cv::line(image, {lx(210),210}, {lx(475),475}, cv::Scalar(255,255,255), 8);
  for (int y = 210; y < 470; y += 58)
    cv::line(image, {rx(y),y}, {rx(std::min(y+30,475)), std::min(y+30,475)}, cv::Scalar(0,230,230), 8);
  adas::RoadGeometry g;
  auto lane = g.analyze_lane(image, stamp(1,1)).lane;
  check(lane.left_valid && lane.right_valid, "perspective lane boundaries fit");
  check(lane.left_type == adas::LineType::Solid && lane.right_type == adas::LineType::Dashed, "occupancy follows sloping solid and dashed boundaries");
  check(lane.left_top_x > lane.left_x + 70 && lane.right_x > lane.right_top_x + 70,
        "lane endpoints represent perspective slopes in pixels");
  check(std::abs(lane.center_offset) < 0.12 && lane.left_conf <= 1 && lane.right_conf <= 1, "lane offset and confidence normalized");
  image.setTo(cv::Scalar(45,45,45));
  for (int y=270;y<475;y+=40) cv::rectangle(image, {80,y,480,13}, cv::Scalar(255,255,255), -1);
  lane = g.analyze_lane(image, stamp(2,2)).lane;
  check(!lane.left_valid && !lane.right_valid, "horizontal zebra bars do not form lane boundaries");
}
cv::Mat sparse_perspective_lane(int strokes = 3) {
  cv::Mat image(720,1280,CV_8UC3,cv::Scalar(45,45,45));
  auto x = [](double y) { return cvRound(1280*(.46+1.42*(y/720-.60))); };
  const std::array<std::array<int,3>,3> marks = {{{457,464,3},{486,500,5},{533,569,8}}};
  for (int i=3-strokes;i<3;++i)
    cv::line(image,{x(marks[i][0]),marks[i][0]},{x(marks[i][1]),marks[i][1]},
             cv::Scalar(255,255,255),marks[i][2]);
  return image;
}
void test_sparse_perspective_lanes() {
  adas::RoadGeometry g;
  auto image=sparse_perspective_lane();
  const auto lane=g.analyze_lane(image,stamp(1,1)).lane;
  check(lane.right_valid && lane.right_type==adas::LineType::Dashed,
        "three narrow perspective dashes survive bounded image analysis");
  check(lane.right_x>1200 && lane.right_top_x<lane.right_x-400,
        "wide perspective boundary retains native extrapolated coordinates");
  auto thick=image.clone();
  auto thick_x=[](int y){return cvRound(1280*(.46+1.42*(double(y)/720-.60)));};
  cv::line(thick,{thick_x(533),533},{thick_x(569),569},cv::Scalar(255,255,255),20);
  check(g.analyze_lane(thick,stamp(2,2)).lane.right_type==adas::LineType::Dashed,
        "wide near paint edges still anchor independent far perspective dashes");
  const auto two=g.analyze_lane(sparse_perspective_lane(2),stamp(2,2)).lane;
  check(two.right_type==adas::LineType::Unknown,
        "two isolated strokes do not establish a dashed boundary");
  image.setTo(cv::Scalar(45,45,45));
  cv::line(image,{580,440},{270,690},cv::Scalar(0,230,230),12);
  for (int i=0;i<4;++i)
    cv::line(image,{850+40*i,360},{900+40*i,325},cv::Scalar(255,255,255),5);
  const auto clutter=g.analyze_lane(image,stamp(3,3)).lane;
  check(clutter.left_valid && clutter.left_type==adas::LineType::Solid &&
        std::abs(clutter.left_x-234)<35,
        "distant roadside segment does not distort the supported ego boundary");
  image.setTo(cv::Scalar(45,45,45));
  cv::line(image,{580,440},{270,690},cv::Scalar(0,230,230),12);
  cv::rectangle(image,{350,520,180,90},cv::Scalar(45,45,45),-1);
  const auto occluded=g.analyze_lane(image,stamp(4,4)).lane;
  check(occluded.left_type==adas::LineType::Unknown,
        "one broad occlusion of a solid marking does not imply dashes");
  for (double period:{70.,95.,125.}) {
    image.setTo(cv::Scalar(45,45,45));
    std::vector<cv::Point> curved;
    for (int y=440;y<=690;++y)
      curved.emplace_back(cvRound(580-1.2*(y-440)+24*std::sin((y-440)*2*CV_PI/period)),y);
    cv::polylines(image,curved,false,cv::Scalar(0,230,230),8);
    const auto continuous=g.analyze_lane(image,stamp(5,5)).lane;
    check(continuous.left_type!=adas::LineType::Dashed,
          "sampling a connected curved solid marking cannot create dashed evidence");
  }
  image=sparse_perspective_lane();
  auto rx=[](int y){return cvRound(1280*(.46+1.42*(double(y)/720-.60)));};
  for (auto gap: {std::make_pair(464,486),std::make_pair(500,533)}) {
    const int middle=(gap.first+gap.second)/2;
    const std::vector<cv::Point> bend={{rx(gap.first),gap.first},{rx(middle)+60,middle},
                                      {rx(gap.second),gap.second}};
    cv::polylines(image,bend,false,cv::Scalar(255,255,255),6);
  }
  check(g.analyze_lane(image,stamp(6,6)).lane.right_type!=adas::LineType::Dashed,
        "one connected bent marking is not three independent painted dashes");
  image.setTo(cv::Scalar(45,45,45));
  for (auto span:{std::make_pair(440,510),std::make_pair(521,591),std::make_pair(602,690)})
    cv::line(image,{580-(span.first-440),span.first},{580-(span.second-440),span.second},
             cv::Scalar(255,255,255),8);
  check(g.analyze_lane(image,stamp(7,7)).lane.left_type!=adas::LineType::Solid,
        "separated paint cannot become solid through aggregate row occupancy");
  image.setTo(cv::Scalar(45,45,45));
  cv::rectangle(image,{640,302,640,140},cv::Scalar(190,190,190),-1);
  cv::line(image,{660,440},{910,690},cv::Scalar(190,190,190),8);
  const auto fused=g.analyze_lane(image,stamp(8,8)).lane;
  check(fused.right_type==adas::LineType::Unknown,
        "paint fused into a broad bright surface is not reliable line type evidence");
}
cv::Mat rgb565_roundtrip(const cv::Mat &image) {
  cv::Mat packed,result;
  cv::cvtColor(image,packed,cv::COLOR_BGR2BGR565);
  cv::cvtColor(packed,result,cv::COLOR_BGR5652BGR);
  return result;
}
void test_lane_evidence_and_ego_selection() {
  adas::RoadGeometry g;
  auto right_x=[](int y) { return cvRound(1280*(.50+1.02*(double(y)/720-.60))); };
  auto left_x=[](int y) { return cvRound(1280*(.46-.85*(double(y)/720-.60))); };
  auto dashed=[&](cv::Mat &image,cv::Scalar paint) {
    for (auto span:{std::make_pair(450,459),std::make_pair(482,500),std::make_pair(548,586)})
      cv::line(image,{right_x(span.first),span.first},{right_x(span.second),span.second},paint,6);
  };
  cv::Mat image(720,1280,CV_8UC3,cv::Scalar(190,190,190));
  dashed(image,cv::Scalar(235,235,235));
  auto lane=g.analyze_lane(rgb565_roundtrip(image),stamp(1,1)).lane;
  check(lane.right_valid && lane.right_type==adas::LineType::Dashed,
        "three white dashes remain independent on bright concrete after RGB565");
  image.setTo(cv::Scalar(45,45,45));
  cv::line(image,{right_x(530),530},{right_x(710),710},cv::Scalar(255,255,255),9);
  lane=g.analyze_lane(image,stamp(2,2)).lane;
  check(lane.right_type==adas::LineType::Unknown,
        "one long near dash cannot fill its own Hough interval and become solid");
  image.setTo(cv::Scalar(45,45,45));
  dashed(image,cv::Scalar(255,255,255));
  auto outer_x=[](int y) { return cvRound(1280*(.55+1.75*(double(y)/720-.60))); };
  cv::line(image,{outer_x(432),432},{outer_x(690),690},cv::Scalar(255,255,255),9);
  lane=g.analyze_lane(image,stamp(3,3)).lane;
  check(lane.right_valid && lane.right_type==adas::LineType::Dashed &&
        std::abs(lane.right_x-right_x(719))<35,
        "longer adjacent solid cannot replace the nearest dashed ego boundary");
  image.setTo(cv::Scalar(45,45,45));
  cv::line(image,{outer_x(432),432},{outer_x(690),690},cv::Scalar(255,255,255),9);
  lane=g.analyze_lane(image,stamp(4,4)).lane;
  check(!lane.right_valid && lane.right_type==adas::LineType::Unknown,
        "a strong outer lane alone cannot stand in for missing ego boundary evidence");
  image.setTo(cv::Scalar(190,190,190));
  cv::line(image,{left_x(430),430},{left_x(719),719},cv::Scalar(0,230,230),9);
  cv::line(image,{right_x(430),430},{right_x(719),719},cv::Scalar(235,235,235),9);
  lane=g.analyze_lane(rgb565_roundtrip(image),stamp(5,5)).lane;
  check(lane.left_type==adas::LineType::Solid && lane.right_type==adas::LineType::Solid,
        "complete white and yellow solid boundaries survive local contrast filtering");
  image.setTo(cv::Scalar(45,45,45));
  for (auto span:{std::make_pair(450,459),std::make_pair(482,500),std::make_pair(548,586)}) {
    std::vector<cv::Point> curve;
    for (int y=span.first;y<=span.second;++y)
      curve.emplace_back(right_x(y)+cvRound(.0015*(590-y)*(590-y)),y);
    cv::polylines(image,curve,false,cv::Scalar(255,255,255),6);
  }
  lane=g.analyze_lane(rgb565_roundtrip(image),stamp(6,6)).lane;
  check(lane.right_type==adas::LineType::Dashed,
        "three independent dashes on a gentle curve are sampled along the paint");
  image.setTo(cv::Scalar(45,45,45));
  // At 480px analysis width the far dash is only a few rows along this
  // steep perspective line. Its small transverse
  // offset must be measured perpendicular to the hypothesis, like Hough.
  for (auto span:{std::make_pair(458,465),std::make_pair(482,493),std::make_pair(539,591)}) {
    const int offset=span.first<470?-13:0;
    cv::line(image,{right_x(span.first)+offset,span.first},
             {right_x(span.second)+offset,span.second},cv::Scalar(255,255,255),3);
  }
  lane=g.analyze_lane(rgb565_roundtrip(image),stamp(7,7)).lane;
  check(lane.right_type==adas::LineType::Dashed,
        "a short far perspective dash retains spatial paint evidence after scaling");
  cv::Mat speck(270,480,CV_8UC3,cv::Scalar(45,45,45));
  auto steep_x=[](int y) { return cvRound(1.8*y-46); };
  for (auto span:{std::make_pair(180,186),std::make_pair(202,222)})
    cv::line(speck,{steep_x(span.first),span.first},{steep_x(span.second),span.second},
             cv::Scalar(255,255,255),3);
  speck.at<cv::Vec3b>(170,steep_x(170))={255,255,255};
  check(g.analyze_lane(speck,stamp(8,8)).lane.right_type==adas::LineType::Unknown,
        "a single bright pixel cannot turn two paint strokes into dashes");
  speck.at<cv::Vec3b>(170,steep_x(170)+1)={255,255,255};
  check(g.analyze_lane(speck,stamp(9,9)).lane.right_type==adas::LineType::Unknown,
        "a two-pixel transverse reflector cannot supply the third painted dash");
  image.setTo(cv::Scalar(190,190,190));
  for (auto span:{std::make_pair(450,465),std::make_pair(495,520),std::make_pair(565,625)}) {
    cv::line(image,{right_x(span.first),span.first},{right_x(span.second),span.second},
             cv::Scalar(235,235,235),6);
    for (int y=span.first+7;y<span.second-3;y+=12)
      cv::line(image,{right_x(y),y},{right_x(y+3),y+3},cv::Scalar(196,196,196),6);
  }
  check(g.analyze_lane(rgb565_roundtrip(image),stamp(9,9)).lane.right_type==adas::LineType::Dashed,
        "contrast fragments inside three independent paint identities retain dashed evidence");
  image=sparse_perspective_lane();
  auto perspective_x=[](int y) { return cvRound(1280*(.46+1.42*(double(y)/720-.60))); };
  cv::line(image,{perspective_x(533),533},{perspective_x(569),569},cv::Scalar(0,230,230),8);
  check(g.analyze_lane(rgb565_roundtrip(image),stamp(10,10)).lane.right_type==adas::LineType::Unknown,
        "two white objects and one yellow marking cannot establish a dashed boundary");
  image=sparse_perspective_lane();
  cv::line(image,{perspective_x(552),552},{perspective_x(569),569},cv::Scalar(0,230,230),8);
  check(g.analyze_lane(rgb565_roundtrip(image),stamp(11,11)).lane.right_type==adas::LineType::Unknown,
        "one ambiguous white-yellow paint identity cannot complete white dashed evidence");
  // All three bright sections still belong to one narrow raw-color marking.
  // Filtering weak/shadowed connectors must not invent independent paint.
  for (bool bent:{false,true}) {
    image.setTo(cv::Scalar(190,190,190));
    std::vector<cv::Point> connector;
    for (int y=450;y<=586;++y)
      connector.emplace_back(right_x(y)+(bent?cvRound(18*std::sin((y-450)*CV_PI/25)):0),y);
    cv::polylines(image,connector,false,cv::Scalar(196,196,196),4);
    dashed(image,cv::Scalar(235,235,235));
    lane=g.analyze_lane(rgb565_roundtrip(image),stamp(bent?7:6,bent?7:6)).lane;
    check(lane.right_type!=adas::LineType::Dashed,
          "weak or curved connected white paint cannot be split into three dashes");
  }
  image.setTo(cv::Scalar(45,45,45));
  // Two inward crossing boundaries can pass old bottom-side checks.
  cv::line(image,{170,440},{600,710},cv::Scalar(255,255,255),9);
  cv::line(image,{1110,440},{680,710},cv::Scalar(255,255,255),9);
  lane=g.analyze_lane(image,stamp(8,8)).lane;
  check(!(lane.left_valid && lane.right_valid),
        "converging toward the vehicle does not produce a valid ego corridor");
  image.setTo(cv::Scalar(45,45,45));
  auto boundary_x=[](int y,bool left) { return cvRound(640+(left?-.9:1.4)*(y-430)); };
  cv::line(image,{boundary_x(440,true),440},{boundary_x(710,true),710},cv::Scalar(255,255,255),8);
  for (auto span:{std::make_pair(460,475),std::make_pair(510,540),std::make_pair(600,650)})
    cv::line(image,{boundary_x(span.first,false),span.first},
             {boundary_x(span.second,false),span.second},cv::Scalar(255,255,255),6);
  // Aligned scenery above the true vanishing point cannot extend the drawn
  // corridor into an X, even when it does not change the selected near lines.
  for (bool left:{true,false})
    cv::line(image,{boundary_x(340,left),340},{boundary_x(350,left),350},cv::Scalar(255,255,255),3);
  lane=g.analyze_lane(rgb565_roundtrip(image),stamp(12,12)).lane;
  check(lane.left_valid && lane.right_valid && lane.left_top_x<lane.right_top_x && lane.top_y>425,
        "visible lane endpoints remain ordered below the vanishing point despite aligned scenery");
}
cv::Mat native_sparse_white_lane(bool far_paint = true) {
  cv::Mat image(720,1280,CV_8UC3,cv::Scalar(45,45,45));
  auto x=[](int y) { return cvRound(1280*(.50+1.02*(double(y)/720-.60))); };
  for (auto span:{std::make_pair(490,505),std::make_pair(550,605)})
    cv::line(image,{x(span.first),span.first},{x(span.second),span.second},cv::Scalar(255,255,255),
             span.first<520?5:9);
  if (far_paint) cv::line(image,{x(458),458},{x(461),461},cv::Scalar(255,255,255),1);
  return image;
}
void test_native_lane_evidence() {
  adas::RoadGeometry g;
  auto x=[](int y) { return cvRound(1280*(.50+1.02*(double(y)/720-.60))); };
  auto base=native_sparse_white_lane(false),image=native_sparse_white_lane();
  const auto two=g.analyze_lane(rgb565_roundtrip(base),stamp(1,1)).lane;
  auto packed=rgb565_roundtrip(image);cv::Mat work;cv::resize(packed,work,{480,270},0,0,cv::INTER_AREA);
  const auto coarse=g.analyze_lane(work,stamp(2,2)).lane;
  const auto native=g.analyze_lane(packed,stamp(3,3)).lane;
  check(two.right_type==adas::LineType::Unknown && coarse.right_type==adas::LineType::Unknown &&
        native.right_type==adas::LineType::Dashed,
        "native pixels corroborate a four-row white dash lost during bounded analysis");
  check(close(native.right_x,coarse.right_x*1280/480,.01) &&
        close(native.right_top_x,coarse.right_top_x*1280/480,.01) &&
        close(native.top_y,coarse.top_y*720/270,.01),
        "native paint corroboration preserves the selected coarse geometry");
  for (int height:{2,3,4}) {
    image=base.clone();
    cv::rectangle(image,{x(459)-6,459,12,height},cv::Scalar(235,235,235),-1);
    check(g.analyze_lane(rgb565_roundtrip(image),stamp(3,3)).lane.right_type==adas::LineType::Unknown,
          "a transverse native reflector cannot supply a third painted dash");
  }
  image=native_sparse_white_lane();
  for (auto span:{std::make_pair(461,490),std::make_pair(505,550)}) {
    std::vector<cv::Point> connector={{x(span.first),span.first},{1200,(span.first+span.second)/2},
                                      {x(span.second),span.second}};
    cv::polylines(image,connector,false,cv::Scalar(255,255,255),1);
  }
  check(g.analyze_lane(rgb565_roundtrip(image),stamp(4,4)).lane.right_type!=adas::LineType::Dashed,
        "native identities cut by the local ROI edge cannot become three dashes");
  image.setTo(cv::Scalar(190,190,190));
  cv::line(image,{x(450),450},{x(610),610},cv::Scalar(196,196,196),1);
  for (auto span:{std::make_pair(458,461),std::make_pair(490,505),std::make_pair(550,605)})
    cv::line(image,{x(span.first),span.first},{x(span.second),span.second},cv::Scalar(235,235,235),
             span.first<480?1:span.first<520?5:9);
  check(g.analyze_lane(rgb565_roundtrip(image),stamp(5,5)).lane.right_type!=adas::LineType::Dashed,
        "one native weak identity network remains one physical marking");
  cv::resize(native_sparse_white_lane(),image,{2560,1440},0,0,cv::INTER_NEAREST);
  check(g.analyze_lane(rgb565_roundtrip(image),stamp(6,6)).lane.right_type==adas::LineType::Unknown,
        "an oversized native ROI fails closed within the fixed pixel budget");
  image=native_sparse_white_lane();
  for (int y=433;y<647;y+=5) for (int xx=550;xx<1050;xx+=5)
    image.at<cv::Vec3b>(y,xx)={255,255,255};
  check(g.analyze_lane(rgb565_roundtrip(image),stamp(7,7)).lane.right_type==adas::LineType::Unknown,
        "excessive native components fail closed within the component budget");
}
} // namespace
int main() {
  test_calibration(); test_lane_only_contract(); test_lanes(); test_sparse_perspective_lanes();
  test_lane_evidence_and_ego_selection(); test_native_lane_evidence();
  std::cout << "road_geometry_test: " << failures << " failures\n";
  return failures ? 1 : 0;
}
