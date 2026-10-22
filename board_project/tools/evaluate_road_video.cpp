// Offline geometry/rule diagnostics. No RKNN or hardware access; target
// observations are explicitly valid-empty substitutes for isolated rule tests.
#include "core/adas_engine.h"
#include "core/road_geometry.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <array>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>

int main(int argc,char **argv) {
  if(argc<3||argc>5) {
    std::cerr<<"Usage: evaluate_road_video LOCAL_VIDEO OUTPUT_DIR [frame_stride=5] [sample_frames_csv]\n";
    return 2;
  }
  try {
    int stride=5;
    if(argc>=4){size_t used=0;stride=std::stoi(argv[3],&used);if(argv[3][used]!='\0'||stride<1||stride>1000)return 2;}
    std::set<unsigned> pictures;
    if(argc==5) {
      std::istringstream values(argv[4]);std::string value;
      while(std::getline(values,value,',')) {
        size_t used=0;const int frame=std::stoi(value,&used);
        if(used!=value.size()||frame<0||frame%stride)return 2;
        pictures.insert(unsigned(frame));
      }
    }
    if(!std::filesystem::is_regular_file(argv[1]))return 2;
    cv::setNumThreads(1);
    cv::VideoCapture source(argv[1]);if(!source.isOpened())return 2;
    const double fps=source.get(cv::CAP_PROP_FPS);
    if(!std::isfinite(fps)||fps<=0)return 2;
    const std::filesystem::path output(argv[2]);std::filesystem::create_directories(output);
    std::ofstream csv(output/"samples.csv");if(!csv)return 2;
    csv<<"frame,time,left_valid,left_type,right_valid,right_type,nav_detected,nav_direction,"
         "rule_only_left_allowed,rule_only_right_allowed,forced_left_alarm,forced_right_alarm,ms,"
         "left_x70,left_x80,left_x90,right_x70,right_x80,right_x90,left_conf,right_conf,"
         "left_top_x,right_top_x,top_y,left_bottom_x,right_bottom_x,bottom_y\n";
    adas::RoadGeometry geometry;adas::AdasEngine engine;adas::EgoState ego;
    ego.input_layout=adas::Layout::Single;
    std::array<unsigned,3> left{},right{};unsigned decoded=0,samples=0,allow_left=0,allow_right=0,nav_count=0;
    cv::Mat frame;std::vector<double> times;
    while(source.read(frame)) {
      const unsigned index=decoded++;
      if(index%unsigned(stride))continue;
      // Match RGB565 precision of the actual file/PCIe image path.
      cv::Mat packed,bgr;cv::cvtColor(frame,packed,cv::COLOR_BGR2BGR565);
      cv::cvtColor(packed,bgr,cv::COLOR_BGR5652BGR);
      const double time=index/fps;
      const adas::Stamp stamp{1,uint64_t(index)+1,time,time,false};
      auto road=geometry.analyze(bgr,stamp,{},adas::Roi{0,0,.32,.3});
      adas::ObjectObservation empty_front;empty_front.stamp=stamp;
      empty_front.components=adas::ObjectObservation::Traffic;
      empty_front.traffic_status=adas::Status::Valid;empty_front.width=bgr.cols;empty_front.height=bgr.rows;
      engine.ingest(empty_front,time);engine.ingest(road,time);
      ego.signal=adas::TurnSignal::Left;ego.force_lane_change=adas::TurnSignal::None;
      const auto l=engine.evaluate(ego,time);
      ego.signal=adas::TurnSignal::Right;const auto r=engine.evaluate(ego,time);
      ego.force_lane_change=adas::TurnSignal::Left;const auto forced_l=engine.evaluate(ego,time);
      ego.force_lane_change=adas::TurnSignal::Right;const auto forced_r=engine.evaluate(ego,time);
      ++left.at(size_t(road.lane.left_type));++right.at(size_t(road.lane.right_type));
      ++samples;allow_left+=l.lane_change_allowed;allow_right+=r.lane_change_allowed;nav_count+=road.nav.detected;
      times.push_back(road.process_ms);
      const auto &lane=road.lane;
      const auto line_x=[&](bool left,double fraction) {
        const double top=left?lane.left_top_x:lane.right_top_x;
        const double bottom=left?lane.left_x:lane.right_x;
        return top+(fraction*(bgr.rows-1)-lane.top_y)*(bottom-top)/(lane.bottom_y-lane.top_y);
      };
      csv<<index<<','<<time<<','<<road.lane.left_valid<<','<<int(road.lane.left_type)<<','
         <<road.lane.right_valid<<','<<int(road.lane.right_type)<<','<<road.nav.detected<<','<<int(road.nav.dir)<<','
         <<l.lane_change_allowed<<','<<r.lane_change_allowed<<','<<forced_l.alarm_illegal_lane_change<<','
         <<forced_r.alarm_illegal_lane_change<<','<<road.process_ms;
      for(bool side:{true,false})for(double row:{.7,.8,.9})csv<<','<<line_x(side,row);
      csv<<','<<lane.left_conf<<','<<lane.right_conf<<','<<lane.left_top_x<<','<<lane.right_top_x
         <<','<<lane.top_y<<','<<lane.left_x<<','<<lane.right_x<<','<<lane.bottom_y<<'\n';
      if(pictures.count(index)) {
        cv::Mat raw=bgr.clone();
        for(int x=0;x<bgr.cols;x+=80) {
          cv::line(raw,{x,0},{x,bgr.rows-1},{90,90,90},1);
          cv::putText(raw,std::to_string(x),{x+2,30},cv::FONT_HERSHEY_SIMPLEX,.6,{255,100,255},1);
        }
        for(double row:{.7,.8,.9}) {
          const int y=cvRound(row*(bgr.rows-1));cv::line(raw,{0,y},{bgr.cols-1,y},{80,80,80},1);
          cv::putText(raw,std::to_string(y),{4,y-4},cv::FONT_HERSHEY_SIMPLEX,.6,{255,100,255},1);
        }
        cv::imwrite((output/("raw-"+std::to_string(index)+".png")).string(),raw);
        cv::Mat overlay=bgr.clone();
        for(bool side:{true,false}) {
          if(!(side?lane.left_valid:lane.right_valid))continue;
          const auto type=side?lane.left_type:lane.right_type;
          const cv::Scalar color=type==adas::LineType::Dashed?cv::Scalar(0,255,0):
            type==adas::LineType::Solid?cv::Scalar(0,220,255):cv::Scalar(150,150,150);
          cv::line(overlay,{cvRound(side?lane.left_top_x:lane.right_top_x),cvRound(lane.top_y)},
            {cvRound(side?lane.left_x:lane.right_x),cvRound(lane.bottom_y)},color,3);
        }
        cv::putText(overlay,"frame "+std::to_string(index)+" L="+std::to_string(int(lane.left_type))+" R="+
          std::to_string(int(lane.right_type)),{12,32},cv::FONT_HERSHEY_SIMPLEX,.8,{255,50,255},2);
        cv::imwrite((output/("overlay-"+std::to_string(index)+".png")).string(),overlay);
      }
    }
    csv.close();if(!csv||!samples)return 2;
    cv::FileStorage report((output/"summary.json").string(),cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
    if(!report.isOpened())return 2;
    report<<"input"<<std::filesystem::absolute(argv[1]).string()<<"fps"<<fps<<"stride"<<stride
      <<"decoded_frames"<<int(decoded)<<"geometry_samples"<<int(samples)
      <<"inference_backend"<<"not-run: valid-empty Traffic substitutes for isolated geometry/rule tests"
      <<"accuracy_claim"<<"none: no frame-level ground truth or RKNN inference"
      <<"rgb565_roundtrip"<<true
      <<"left_unknown"<<int(left[0])<<"left_solid"<<int(left[1])<<"left_dashed"<<int(left[2])
      <<"right_unknown"<<int(right[0])<<"right_solid"<<int(right[1])<<"right_dashed"<<int(right[2])
      <<"rule_only_left_allowed"<<int(allow_left)<<"rule_only_right_allowed"<<int(allow_right)
      <<"navigation_samples"<<int(nav_count);
    double total=0;for(double t:times)total+=t;std::sort(times.begin(),times.end());
    report<<"geometry_mean_ms"<<total/times.size()<<"geometry_p95_ms"<<times[size_t(std::ceil(times.size()*.95))-1]
      <<"geometry_max_ms"<<times.back();
    std::cout<<"Decoded "<<decoded<<" frames; geometry samples "<<samples
      <<"; left unknown/solid/dashed "<<left[0]<<'/'<<left[1]<<'/'<<left[2]
      <<"; right "<<right[0]<<'/'<<right[1]<<'/'<<right[2]<<"\nNo model accuracy claim.\n";
  } catch(const std::exception &error) {std::cerr<<error.what()<<'\n';return 2;}
  return 0;
}
