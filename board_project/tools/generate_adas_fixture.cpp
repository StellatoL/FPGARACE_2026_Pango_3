// Synthetic replay fixture tests scheduling/rendering, not neural accuracy.
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <filesystem>
#include <iostream>

int main(int argc,char **argv) {
  const std::string folder=argc>1?argv[1]:"/tmp/adas-fixtures";
  std::filesystem::create_directories(folder);
  cv::VideoWriter video(folder+"/mosaic.avi",cv::VideoWriter::fourcc('M','J','P','G'),30,{1280,720});
  if(!video.isOpened())return 1;
  cv::Mat first;
  for(int frame=0;frame<180;++frame) {
    cv::Mat canvas(720,1280,CV_8UC3,cv::Scalar(25,30,35));
    for(int v=0;v<4;++v) {
      cv::Mat tile=canvas(cv::Rect((v%2)*640,(v/2)*360,640,360));
      tile.rowRange(0,140).setTo(cv::Scalar(90,75,55));
      const cv::Point left_bottom(110,359),left_top(285,150),right_bottom(530,359),right_top(355,150);
      cv::line(tile,left_top,left_bottom,cv::Scalar(235,235,235),7);
      for(int y=160-(frame*3)%45;y<360;y+=45) {
        int end=std::min(y+24,359);if(y<145)continue;
        auto x=[](int py){return 355+(py-150)*175/209;};
        cv::line(tile,{x(y),y},{x(end),end},cv::Scalar(50,220,235),6);
      }
      for(int i=0;i<70;++i) {
        int y=155+(i*37+frame*2)%200;int x=200+(i*53)%240;
        cv::circle(tile,{x,y},1,cv::Scalar(90+(i%4)*20,90,90),-1);
      }
      if(v==0) {
        std::vector<cv::Point> arrow{{90,25},{115,53},{103,53},{103,87},{77,87},{77,53},{65,53}};
        cv::fillPoly(tile,std::vector<std::vector<cv::Point>>{arrow},cv::Scalar(255,120,0));
      }
      cv::putText(tile,std::string("VIEW ")+std::to_string(v),{460,40},cv::FONT_HERSHEY_SIMPLEX,.6,cv::Scalar(240,240,240),1);
    }
    if(frame==0)first=canvas.clone();video.write(canvas);
  }
  video.release();
  cv::imwrite(folder+"/mosaic.png",first);
  cv::Mat front;cv::resize(first(cv::Rect(0,0,640,360)),front,cv::Size(1280,720));
  cv::imwrite(folder+"/front.png",front);
  std::cout<<folder<<"/mosaic.avi\n";
}
