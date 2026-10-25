// Offline integration diagnostic only. Not linked into the production app.
// Tests the same owned RGB565 -> strict RGA -> RKNN -> decoder as live ADAS.
#include "inference/neural_models.h"
#include <QCoreApplication>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <set>
int main(int argc,char **argv) {
  QCoreApplication app(argc,argv);
  if(argc!=3){std::cerr<<"usage: smoke_scene MODEL_ROOT IMAGE\n";return 2;}
  cv::Mat image=cv::imread(argv[2]);if(image.empty())return 2;
  adas::NeuralModels models;
  if(!models.initialize(QString::fromLocal8Bit(argv[1]))){std::cerr<<models.description().toStdString()<<'\n';return 3;}
  std::cout<<models.description().toStdString()<<'\n';
  const std::set<std::string> labels={"person","vehicle","crosswalk","light_red","light_green","cone","barrier","debris"};
  for(auto layout:{adas::Layout::Single,adas::Layout::Mosaic}) {
    cv::Mat resized,rgb565;cv::resize(image,resized,{1280,720});
    cv::cvtColor(resized,rgb565,cv::COLOR_BGR2BGR565); // Offline fixture preparation, never production fallback.
    auto pixels=std::make_shared<adas::FrameStorage>(1280*720*2);
    std::copy(rgb565.datastart,rgb565.dataend,pixels->bytes.begin());
    const double now=adas::monotonic_seconds();
    adas::FramePacket packet{pixels,{1,1,now,now,false},1280,720,2560,layout};
    std::array<adas::Roi,3> lights;lights.fill({0,0,1,.5});
    for(int view=0;view<(layout==adas::Layout::Single?1:4);++view) {
      const auto r=models.scene(packet,static_cast<adas::ViewId>(view),lights,{0,0,.32,.30});
      const auto &o=r.vision.observation;
      if(o.traffic_status!=adas::Status::Valid||o.vehicle_status!=adas::Status::Valid||o.roadblock_status!=adas::Status::Valid){std::cerr<<r.vision.error_message<<'\n';return 4;}
      for(const auto &box:r.vision.overlays)
        if(!labels.count(box.label)||box.left<0||box.top<0||box.right>o.width||box.bottom>o.height)return 5;
      std::cout<<"layout="<<int(layout)<<" view="<<view<<" size="<<o.width<<'x'<<o.height
               <<" boxes="<<r.vision.overlays.size()<<" inference_ms="<<o.infer_ms<<'\n';
    }
    if(!models.lane_configured()&&models.lane(packet).status!=adas::Status::Missing)return 6;
  }
  std::cout<<"Actual RGA/RKNN single and four-view interface smoke passed; no accuracy claim\n";
}
