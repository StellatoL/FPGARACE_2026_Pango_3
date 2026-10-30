#include "core/road_geometry.h"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>
int main(){cv::setNumThreads(1);adas::RoadGeometry g;int failures=0;
 auto check=[&](bool ok,const char *name){std::cout<<(ok?"PASS ":"FAIL ")<<name<<'\n';failures+=!ok;};
 auto x=[](int y,bool left){return cvRound((left?580:700)+(left?-.8:.8)*(y-430));};
 for(int level:{120,145,160}){
  cv::Mat bgr(720,1280,CV_8UC3,cv::Scalar(65,65,65));
  cv::line(bgr,{x(430,true),430},{x(719,true),719},cv::Scalar(level,level,level),9);
  cv::line(bgr,{x(430,false),430},{x(719,false),719},cv::Scalar(level,level,level),9);
  for(bool left:{true,false})cv::line(bgr,{x(560,left),560},{x(630,left),630},cv::Scalar(235,235,235),9);
  cv::Mat packed,img;cv::cvtColor(bgr,packed,cv::COLOR_BGR2BGR565);cv::cvtColor(packed,img,cv::COLOR_BGR5652BGR);
  const auto lane=g.analyze_lane(img,{1,1,0,0,true}).lane;
  check(lane.left_valid&&lane.right_valid&&lane.left_type==adas::LineType::Solid&&lane.right_type==adas::LineType::Solid,"selected solid boundary is corroborated through shaded paint");
 }
 for(int level:{120,145}){
  cv::Mat image(720,1280,CV_8UC3,cv::Scalar(65,65,65));
  for(auto span:{std::make_pair(450,465),std::make_pair(495,520),std::make_pair(565,625)})
   cv::line(image,{x(span.first,false),span.first},{x(span.second,false),span.second},cv::Scalar(span.first<480?level:235,span.first<480?level:235,span.first<480?level:235),7);
  check(g.analyze_lane(image,{1,1,0,0,true}).lane.right_type==adas::LineType::Dashed,"native shaded third dash corroborates two selected bright dashes");
 }
 for(int level:{80,110,140}){
  cv::Mat image(720,1280,CV_8UC3,cv::Scalar(level,level,level));const auto lane=g.analyze_lane(image,{1,1,0,0,true}).lane;
  check(!lane.left_valid&&!lane.right_valid,"uniform low-light asphalt cannot create lane evidence");
 }
 return failures?1:0;
}
