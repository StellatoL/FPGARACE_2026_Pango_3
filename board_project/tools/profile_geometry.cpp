// Profile front CPU work independently from decoding and NPU contention.
#include "core/road_geometry.h"
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <iostream>
#include <vector>
int main(int argc,char **argv) {
  if(argc<2)return 2;
  cv::setNumThreads(1);
  cv::VideoCapture source(argv[1]);if(!source.isOpened())return 2;
  adas::GroundCalibration calibration;
  if(argc>2&&!calibration.load(argv[2],640,360))return 2;
  adas::RoadGeometry geometry;std::vector<double> elapsed;
  for(int i=0;i<90;++i) {
    cv::Mat frame;if(!source.read(frame)||frame.cols<640||frame.rows<360)break;
    auto result=geometry.analyze(frame(cv::Rect(0,0,640,360)),{1,uint64_t(i+1),i/30.,i/30.,false},calibration,{0,0,.32,.3});
    if(i>0)elapsed.push_back(result.process_ms);
  }
  if(elapsed.empty())return 2;
  double total=0;for(auto value:elapsed)total+=value;
  std::sort(elapsed.begin(),elapsed.end());
  std::cout<<"calibrated="<<calibration.valid()<<" samples="<<elapsed.size()<<" mean_ms="<<total/elapsed.size()
    <<" p95_ms="<<elapsed[size_t((elapsed.size()-1)*.95)]<<'\n';
}
