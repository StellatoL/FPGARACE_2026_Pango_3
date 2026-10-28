// Optional hardware regression: ADAS_TEST_RGA=1 tests/run_tests.sh
#include "inference/rga_preprocessor.h"
#include <vector>
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
  vision::RgaPreprocessor rga;assert(!rga.is_available());
  constexpr int width=128,height=96,stride=272;
  std::vector<unsigned char> source(stride*height,0x96);
  for(int y=0;y<height;++y)for(int x=0;x<width;++x){
    const unsigned short p=(((x/8)&31)<<11)|(((x/8+y/8)&63)<<5)|((y/4)&31);
    source[y*stride+2*x]=p&255;source[y*stride+2*x+1]=p>>8;
  }
  cv::Mat unavailable;
  assert(!rga.rgb565_crop_to_rgb_strict(source.data(),width,height,stride,{0,0,64,48},unavailable,64,48,{0,0,64,48}));
  assert(unavailable.empty());
  assert(rga.initialize());assert(rga.is_available());
  auto check=[&](const cv::Rect &crop,int w,int h,const cv::Rect &dst){
    cv::Mat rgb;assert(rga.rgb565_crop_to_rgb_strict(source.data(),width,height,stride,crop,rgb,w,h,dst));
    for(int y=0;y<h;++y)for(int x=0;x<w;++x){
      const auto *pixel=rgb.ptr<unsigned char>(y)+3*x;
      if(x<dst.x||y<dst.y||x>=dst.x+dst.width||y>=dst.y+dst.height){assert(pixel[0]==0&&pixel[1]==0&&pixel[2]==0);continue;}
      const int sx=crop.x+x-dst.x,sy=crop.y+y-dst.y;
      const unsigned p=source[sy*stride+sx*2]|(unsigned(source[sy*stride+sx*2+1])<<8);
      const int expected[]={int(((p>>11)&31)<<3),int(((p>>5)&63)<<2),int((p&31)<<3)};
      for(int c=0;c<3;++c)assert(std::abs(int(pixel[c])-expected[c])<=8);
    }
  };
  check({32,24,96,48},128,64,{16,8,96,48});
  for(int y:{0,48})for(int x:{0,64})check({x,y,64,48},64,48,{0,0,64,48});
  cv::Mat invalid;
  assert(!rga.rgb565_crop_to_rgb_strict(nullptr,width,height,stride,{0,0,64,48},invalid,64,48,{0,0,64,48}));
  assert(!rga.rgb565_crop_to_rgb_strict(source.data(),width,height,width*2-2,{0,0,64,48},invalid,64,48,{0,0,64,48}));
  assert(!rga.rgb565_crop_to_rgb_strict(source.data(),width,height,stride+1,{0,0,64,48},invalid,64,48,{0,0,64,48}));
  for(const cv::Rect &crop:{cv::Rect(-2,0,64,48),cv::Rect(0,-2,64,48),cv::Rect(100,0,64,48),
      cv::Rect(0,70,64,48),cv::Rect(0,0,0,48),cv::Rect(0,0,64,0)})
    assert(!rga.rgb565_crop_to_rgb_strict(source.data(),width,height,stride,crop,invalid,64,48,{0,0,64,48}));
  for(const cv::Rect &destination:{cv::Rect(-2,0,64,48),cv::Rect(0,-2,64,48),cv::Rect(2,0,64,48),
      cv::Rect(0,2,64,48),cv::Rect(0,0,0,48),cv::Rect(0,0,64,0)})
    assert(!rga.rgb565_crop_to_rgb_strict(source.data(),width,height,stride,{0,0,64,48},invalid,64,48,destination));
  assert(invalid.empty()); // Validation fails before any output allocation or blit.
  // Production dual640 letterboxing preserves the FPGA canvas and byte stride.
  constexpr int canvas_w=1280,canvas_h=720,canvas_stride=canvas_w*2+32;
  std::vector<unsigned char> canvas(canvas_stride*canvas_h,0xa5);
  const unsigned short colors[]={0xf800,0x07e0,0x001f,0xffff};
  const int channels[4][3]={{248,0,0},{0,252,0},{0,0,248},{248,252,248}};
  for(int y=0;y<canvas_h;++y)for(int x=0;x<canvas_w;++x) {
    const auto value=colors[(y/360)*2+x/640];
    canvas[y*canvas_stride+2*x]=value&255;canvas[y*canvas_stride+2*x+1]=value>>8;
  }
  auto color_at=[&](const cv::Mat &rgb,int x,int y,int quadrant) {
    const auto pixel=rgb.at<cv::Vec3b>(y,x);
    for(int c=0;c<3;++c)assert(std::abs(int(pixel[c])-channels[quadrant][c])<=8);
  };
  for(int q=0;q<4;++q) {
    cv::Mat rgb;const cv::Rect crop((q%2)*640,(q/2)*360,640,360);
    assert(rga.rgb565_crop_to_rgb_strict(canvas.data(),canvas_w,canvas_h,canvas_stride,crop,rgb,640,640,{0,140,640,360}));
    assert(rgb.cols==640&&rgb.rows==640&&rgb.type()==CV_8UC3);
    for(int y:{0,139,500,639})for(int x:{0,320,639})assert(rgb.at<cv::Vec3b>(y,x)==cv::Vec3b(0,0,0));
    color_at(rgb,320,320,q);
    // The upper-half signal ROI is resized independently of the full view.
    cv::Mat signal;const cv::Rect roi(crop.x,crop.y,640,180);
    assert(rga.rgb565_crop_to_rgb_strict(canvas.data(),canvas_w,canvas_h,canvas_stride,roi,signal,640,640,{0,230,640,180}));
    for(int y:{0,229,410,639})for(int x:{0,320,639})assert(signal.at<cv::Vec3b>(y,x)==cv::Vec3b(0,0,0));
    color_at(signal,320,320,q);
  }
  cv::Mat dual;
  assert(rga.rgb565_crop_to_rgb_strict(canvas.data(),canvas_w,canvas_h,canvas_stride,{0,0,1280,720},dual,640,640,{0,140,640,360}));
  assert(dual.at<cv::Vec3b>(0,320)==cv::Vec3b(0,0,0));
  color_at(dual,160,230,0);color_at(dual,480,230,1);color_at(dual,160,410,2);color_at(dual,480,410,3);
  std::cout<<"Hardware RGA RGB565 colors, padded stride, crop, four regions and padding: PASS\n";
  std::cout<<"Hardware dual640 black padding, signal ROI and single/four-view colors: PASS\n";
}
