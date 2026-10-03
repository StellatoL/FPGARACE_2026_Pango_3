#ifndef SORT_TRACKER_H
#define SORT_TRACKER_H
// Compact SORT tracker — 7D Kalman filter + greedy IoU matching

#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>

namespace sort {

class KalmanBoxTracker {
public:
  float x[7]={}; float P[49]={};
  int hits=0, hit_streak=0, age=0, time_since_update=0;
  static int count; int id;

  KalmanBoxTracker(const float *bbox) { // [x1,y1,x2,y2]
    id=count++;
    float w=bbox[2]-bbox[0], h=bbox[3]-bbox[1];
    x[0]=bbox[0]+w/2; x[1]=bbox[1]+h/2; x[2]=w*h; x[3]=w/std::max(h,1.f);
    for(int i=0;i<7;i++) P[i*7+i]=10.f;
    P[4*7+4]=P[5*7+5]=P[6*7+6]=1000.f;
  }

  void predict() {
    float F[49]={}; for(int i=0;i<7;i++) F[i*7+i]=1.f;
    F[0*7+4]=F[1*7+5]=F[2*7+6]=1.f;
    float xp[7]={};
    for(int i=0;i<7;i++) for(int j=0;j<7;j++) xp[i]+=F[i*7+j]*x[j];
    float Pp[49]={};
    for(int i=0;i<7;i++) for(int k=0;k<7;k++) for(int j=0;j<7;j++) Pp[i*7+j]+=F[i*7+k]*P[k*7+j]*F[j*7+k];
    for(int i=4;i<7;i++) Pp[i*7+i]+=0.01f;
    std::memcpy(x,xp,sizeof(x)); std::memcpy(P,Pp,sizeof(P));
    ++age; ++time_since_update; if(time_since_update>0) hit_streak=0;
  }

  void update(const float *bbox) {
    time_since_update=0; ++hits; ++hit_streak;
    float w=bbox[2]-bbox[0], h=bbox[3]-bbox[1];
    float z[4]={bbox[0]+w/2,bbox[1]+h/2,w*h,w/std::max(h,1.f)};
    float H[28]={}; H[0]=H[8]=H[16]=H[24]=1.f;
    float y[4]={}; for(int i=0;i<4;i++){y[i]=z[i];for(int j=0;j<7;j++)y[i]-=H[i*7+j]*x[j];}
    float S[16]={}; for(int i=0;i<4;i++)for(int j=0;j<4;j++){for(int k=0;k<7;k++)S[i*4+j]+=H[i*7+k]*P[k*7+j]*H[j*7+k];}
    S[0]+=1;S[5]+=1;S[10]+=10;S[15]+=10;
    float det=S[0]*(S[5]*S[10]-S[6]*S[9])-S[1]*(S[4]*S[10]-S[6]*S[8])+S[2]*(S[4]*S[9]-S[5]*S[8]);
    if(std::abs(det)<1e-6f) det=1e-6f;
    float invS[16]={};
    invS[0]=(S[5]*S[10]-S[6]*S[9])/det; invS[1]=(S[2]*S[9]-S[1]*S[10])/det; invS[2]=(S[1]*S[6]-S[2]*S[5])/det;
    invS[4]=(S[6]*S[8]-S[4]*S[10])/det; invS[5]=(S[0]*S[10]-S[2]*S[8])/det; invS[6]=(S[2]*S[4]-S[0]*S[6])/det;
    invS[8]=(S[4]*S[9]-S[5]*S[8])/det; invS[9]=(S[1]*S[8]-S[0]*S[9])/det; invS[10]=(S[0]*S[5]-S[1]*S[4])/det;
    float K[28]={}; for(int i=0;i<7;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)K[i*4+j]+=P[i*7+k]*H[j*7+k]*invS[k*4+j];
    for(int i=0;i<7;i++)for(int j=0;j<4;j++)x[i]+=K[i*4+j]*y[j];
    float newP[49]={}; for(int i=0;i<7;i++)for(int j=0;j<7;j++){newP[i*7+j]=P[i*7+j];for(int k=0;k<4;k++)newP[i*7+j]-=K[i*4+k]*H[k*7+j]*P[0*7+j];}
    std::memcpy(P,newP,sizeof(P));
  }

  void get_bbox(float *b) const { float w=std::sqrt(std::max(1.f,x[2]*x[3])),h=std::max(1.f,x[2]/w); b[0]=x[0]-w/2;b[1]=x[1]-h/2;b[2]=x[0]+w/2;b[3]=x[1]+h/2; }
  void get_center(float *cx,float *cy) const {*cx=x[0];*cy=x[1];}
  void get_velocity(float *vx,float *vy) const {*vx=x[4];*vy=x[5];}
};
int KalmanBoxTracker::count=0;

class Sort {
public:
  struct Track { int id; float x1,y1,x2,y2,cx,cy,vx,vy; };

  Sort(int max_age=2,int min_hits=3,float iou_thr=0.3f):ma_(max_age),mh_(min_hits),iou_(iou_thr){}

  std::vector<Track> update(const float *dets,int n) {
    std::vector<Track> tracks;
    for(auto &t:tr_) t.predict();
    int nd=n, nt=(int)tr_.size();
    std::vector<int> md(nd,-1), mt(nt,-1);

    for(int d=0;d<nd;d++){float bi=0;int bt=-1;
      for(int t=0;t<nt;t++){if(mt[t]>=0)continue;float tb[4];tr_[t].get_bbox(tb);float i=biou(&dets[d*4],tb);if(i>bi&&i>iou_){bi=i;bt=t;}}
      if(bt>=0){md[d]=bt;mt[bt]=d;}}

    for(int d=0;d<nd;d++) if(md[d]>=0) tr_[md[d]].update(&dets[d*4]);
    for(int d=0;d<nd;d++) if(md[d]<0) tr_.emplace_back(&dets[d*4]);

    for(auto &t:tr_){
      if(t.hits<mh_&&(int)tr_.size()>mh_) continue;
      if(t.time_since_update>1) continue;
      Track r;r.id=t.id;float b[4];t.get_bbox(b);r.x1=b[0];r.y1=b[1];r.x2=b[2];r.y2=b[3];t.get_center(&r.cx,&r.cy);t.get_velocity(&r.vx,&r.vy);
      tracks.push_back(r);
    }
    tr_.erase(std::remove_if(tr_.begin(),tr_.end(),[this](auto &t){return t.time_since_update>ma_;}),tr_.end());
    return tracks;
  }
  void reset(){tr_.clear();KalmanBoxTracker::count=0;}
private:
  int ma_,mh_; float iou_; std::vector<KalmanBoxTracker> tr_;
  static float biou(const float *a,const float *b){
    float ix1=std::max(a[0],b[0]),iy1=std::max(a[1],b[1]),ix2=std::min(a[2],b[2]),iy2=std::min(a[3],b[3]);
    float iw=std::max(0.f,ix2-ix1),ih=std::max(0.f,iy2-iy1),ia=iw*ih;
    float aa=(a[2]-a[0])*(a[3]-a[1]),ab=(b[2]-b[0])*(b[3]-b[1]);
    return ia/(aa+ab-ia+1e-6f);
  }
};
} // namespace sort
#endif
