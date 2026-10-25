// Optional hardware regression: ADAS_TEST_RGA=1 tests/run_tests.sh
#include "inference/rga_preprocessor.h"
#include <vector>
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
  vision::RgaPreprocessor rga;assert(rga.initialize());
  constexpr int width=128,height=96,stride=272;
  std::vector<unsigned char> source(stride*height,0x96);
  for(int y=0;y<height;++y)for(int x=0;x<width;++x){
    const unsigned short p=(((x/8)&31)<<11)|(((x/8+y/8)&63)<<5)|((y/4)&31);
    source[y*stride+2*x]=p&255;source[y*stride+2*x+1]=p>>8;
  }
  auto check=[&](const cv::Rect &crop,int w,int h,const cv::Rect &dst){
    cv::Mat rgb;assert(rga.rgb565_crop_to_rgb_strict(source.data(),width,height,stride,crop,rgb,w,h,dst));
    for(int y=0;y<h;++y)for(int x=0;x<w;++x){
      const auto *pixel=rgb.ptr<unsigned char>(y)+3*x;
      if(x<dst.x||y<dst.y||x>=dst.x+dst.width||y>=dst.y+dst.height){assert(pixel[0]==114&&pixel[1]==114&&pixel[2]==114);continue;}
      const int sx=crop.x+x-dst.x,sy=crop.y+y-dst.y;
      const unsigned p=source[sy*stride+sx*2]|(unsigned(source[sy*stride+sx*2+1])<<8);
      const int expected[]={int(((p>>11)&31)<<3),int(((p>>5)&63)<<2),int((p&31)<<3)};
      for(int c=0;c<3;++c)assert(std::abs(int(pixel[c])-expected[c])<=8);
    }
  };
  check({32,24,96,48},128,64,{16,8,96,48});
  for(int y:{0,48})for(int x:{0,64})check({x,y,64,48},64,48,{0,0,64,48});
  cv::Mat invalid;assert(!rga.rgb565_crop_to_rgb_strict(source.data(),width,height,stride,{100,0,64,48},invalid,64,48,{0,0,64,48}));
  std::cout<<"Hardware RGA RGB565 colors, padded stride, crop, four regions and padding: PASS\n";
}
