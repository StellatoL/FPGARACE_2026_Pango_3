#include "core/road_geometry.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace adas {
namespace {
double cross(cv::Point a, cv::Point b) { return double(a.x)*b.y-double(a.y)*b.x; }
bool finite(cv::Point2f p) { return std::isfinite(p.x) && std::isfinite(p.y); }

struct Boundary {
  bool valid = false;
  double slope = 0, intercept = 0;
  int first_y = 0;
  float confidence = 0;
  LineType type = LineType::Unknown;
};
struct Markings {
  cv::Mat mask,labels,stats,identity,raw_labels,raw_stats,yellow;
  int identities = 0,reference_width = 0;
};
bool narrow_component(const cv::Mat &stats,int label,int width) {
  return label && double(stats.at<int>(label,cv::CC_STAT_AREA))/
         stats.at<int>(label,cv::CC_STAT_HEIGHT)<width*.06;
}
Markings markings(const cv::Mat &bgr,int native_width = 0) {
  Markings result;
  result.reference_width=native_width?native_width:bgr.cols;
  cv::Mat hsv,gray,white,yellow,raw,contrast,strong,weak,identity,centroids;
  cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);
  cv::cvtColor(bgr,gray,cv::COLOR_BGR2GRAY);
  cv::inRange(hsv,cv::Scalar(0,0,165),cv::Scalar(179,75,255),white);
  cv::inRange(hsv,cv::Scalar(12,75,100),cv::Scalar(40,255,255),yellow);
  // Horizontal opening removes the broad bright road while preserving narrow
  // paint at each row, including small perspective dashes. Yellow stays an
  // independent color observation rather than requiring a luminance increase.
  const int kernel=std::max(15,(result.reference_width/16)|1);
  cv::morphologyEx(gray,contrast,cv::MORPH_TOPHAT,
                  cv::getStructuringElement(cv::MORPH_RECT,{kernel,1}));
  cv::threshold(contrast,strong,11,255,cv::THRESH_BINARY);
  cv::threshold(contrast,weak,3,255,cv::THRESH_BINARY);
  cv::bitwise_and(strong,white,strong);cv::bitwise_and(weak,white,weak);
  cv::bitwise_or(strong,yellow,result.mask);
  cv::bitwise_or(white,yellow,raw);
  const int top=native_width?0:cvRound(bgr.rows*.42);
  raw.rowRange(0,top).setTo(0);result.mask.rowRange(0,top).setTo(0);
  weak.rowRange(0,top).setTo(0);yellow.rowRange(0,top).setTo(0);
  result.yellow=yellow;
  cv::connectedComponentsWithStats(raw,result.raw_labels,result.raw_stats,centroids,8,CV_32S);
  if (native_width && result.raw_stats.rows>1024) return {};
  cv::bitwise_or(weak,yellow,identity);
  for (int y=top;y<bgr.rows;++y) {
    auto *row=identity.ptr<uchar>(y);
    const auto *labels=result.raw_labels.ptr<int>(y);
    for (int x=0;x<bgr.cols;++x)
      if (narrow_component(result.raw_stats,labels[x],result.reference_width)) row[x]=255;
  }
  // Keep the raw narrow-paint identity even when contrast or shadows split
  // its stronger pixels. A broad concrete component supplies no such identity.
  result.identities=cv::connectedComponents(identity,result.identity,8,CV_32S);
  if (native_width && result.identities>1024) return {};
  cv::connectedComponentsWithStats(result.mask,result.labels,result.stats,centroids,8,CV_32S);
  if (native_width && result.stats.rows>1024) return {};
  return result;
}
bool paint_pixel(const Markings &paint,int y,int x) {
  if (x<0 || x>=paint.mask.cols ||
      !narrow_component(paint.stats,paint.labels.at<int>(y,x),paint.reference_width)) return false;
  const int raw=paint.raw_labels.at<int>(y,x);
  if (narrow_component(paint.raw_stats,raw,paint.reference_width)) return true;
  // A narrow branch attached to a broad bright object remains ambiguous.
  // Recover local contrast on concrete only where the raw bright background
  // itself is broad on both sides of this row, not merely elsewhere in a blob.
  const int side=std::max(3,cvRound(paint.reference_width*.03));
  return raw && x>=side && x+side<paint.mask.cols &&
         paint.raw_labels.at<int>(y,x-side)==raw && paint.raw_labels.at<int>(y,x+side)==raw;
}
int paint_at(const Markings &paint,int y,int x,int radius) {
  for (int d=0;d<=radius;++d) {
    if (paint_pixel(paint,y,x-d)) return x-d;
    if (d && paint_pixel(paint,y,x+d)) return x+d;
  }
  return -1;
}
bool ego_position(const Boundary &boundary,int width,int height,bool left) {
  const double near=(boundary.slope*(height-1)+boundary.intercept)/width;
  const double ahead=(boundary.slope*height*.80+boundary.intercept)/width;
  const double far=(boundary.slope*height*.60+boundary.intercept)/width;
  if (left) return boundary.slope<-.05 && near>-.15 && near<.44 &&
                   ahead>.10 && ahead<.48 && far>.23 && far<.58;
  return boundary.slope>.05 && near>.56 && near<1.15 &&
         ahead>.52 && ahead<.90 && far>.42 && far<.77;
}
Boundary fit_boundary(const Markings &paint,const std::vector<cv::Vec4i> &segments,bool left) {
  Boundary best;
  const auto &mask=paint.mask;
  double best_support=0,best_distance=mask.cols;
  const int radius=std::max(2,mask.cols/240);
  const int min_run=std::max(2,mask.rows/180),min_gap=std::max(3,mask.rows/100);
  const int top=cvRound(mask.rows*.42),near_top=cvRound(mask.rows*.60),bottom=mask.rows-1;
  for (auto seed:segments) {
    const double slope=double(seed[2]-seed[0])/(seed[3]-seed[1]);
    const double intercept=seed[0]-slope*seed[1],normal=std::hypot(1.,slope);
    std::vector<cv::Point2f> points;
    int min_y=mask.rows,max_y=0;
    for (auto segment:segments) {
      // Fit only one supported marking. Roadside edges and other lanes must
      // not pull the line between otherwise unrelated Hough candidates.
      if (std::abs(segment[0]-slope*segment[1]-intercept)/normal>radius*1.5 ||
          std::abs(segment[2]-slope*segment[3]-intercept)/normal>radius*1.5) continue;
      points.emplace_back(segment[0],segment[1]);points.emplace_back(segment[2],segment[3]);
      min_y=std::min({min_y,segment[1],segment[3]});max_y=std::max({max_y,segment[1],segment[3]});
    }
    // One long edge can anchor a hypothesis; line type still requires paint
    // continuity or three independent components, rather than Hough votes.
    if (points.size()<2 || max_y-min_y<mask.rows*.035 || max_y<mask.rows*.62) continue;
    cv::Vec4f line;
    cv::fitLine(points,line,cv::DIST_HUBER,0,.01,.01);
    if (std::abs(line[1])<.1) continue;
    Boundary candidate;
    candidate.slope=line[0]/line[1];candidate.intercept=line[2]-candidate.slope*line[3];
    if (!ego_position(candidate,mask.cols,mask.rows,left)) continue;
    // Hough supplies a hypothesis, not paint length or type. Refit transverse
    // paint centers across the complete visible image, including the far dashes
    // too short to produce their own Hough segment at the bounded resolution.
    points.clear();
    const int fit_radius=cvCeil((radius+1)*std::hypot(1.,candidate.slope));
    for (int y=top;y<=bottom;y+=2) {
      int x=paint_at(paint,y,cvRound(candidate.slope*y+candidate.intercept),fit_radius);
      if (x<0) continue;
      int a=x,b=x;
      const int max_width=std::max(3,cvRound(mask.cols*.06));
      while (a>0 && x-a<max_width && mask.at<uchar>(y,a-1)) --a;
      while (b+1<mask.cols && b-x<max_width && mask.at<uchar>(y,b+1)) ++b;
      if (b-a<max_width) points.emplace_back((a+b)*.5f,float(y));
    }
    if (points.size()<3) continue;
    cv::fitLine(points,line,cv::DIST_HUBER,0,.01,.01);
    if (std::abs(line[1])<.1) continue;
    candidate.slope=line[0]/line[1];candidate.intercept=line[2]-candidate.slope*line[3];
    if (!ego_position(candidate,mask.cols,mask.rows,left)) continue;
    // Match Hough's perpendicular distance. A fixed horizontal radius loses
    // the same paint as perspective makes a boundary steeper in the image.
    const double line_length=std::hypot(1.,candidate.slope);
    const int sample_radius=cvCeil(radius*line_length);
    struct PaintRun { int first,last,label,identity,on,yellow; };
    std::vector<PaintRun> runs;
    std::vector<int> identity_run(paint.identities,-1);
    std::vector<int> component_support(paint.stats.rows,0);
    int on=0,near_on=0,near_visible=0,first=mask.rows,last=0;
    for (int y=top;y<=bottom;++y) {
      const int x=cvRound(candidate.slope*y+candidate.intercept);
      const int sample=paint_at(paint,y,x,sample_radius);
      const int label=sample<0?0:paint.labels.at<int>(y,sample);
      const int identity=sample<0?0:paint.identity.at<int>(y,sample);
      const bool occupied=label!=0;
      if (y>=near_top && x>=0 && x<mask.cols) ++near_visible;
      if (occupied) {
        ++on;first=std::min(first,y);last=y;
        if (y>=near_top) { ++near_on;++component_support[label]; }
        // Strong contrast may fragment one physical marking. Its weaker/raw
        // connected identity owns all of those observations and counts once.
        // Never split a connected curved/shadowed solid into multiple dashes.
        if (identity) {
          int &index=identity_run[identity];
          if (index<0) { index=int(runs.size());runs.push_back({y,y,label,identity,0,0}); }
          auto &run=runs[index];run.last=y;++run.on;
          run.yellow+=paint.yellow.at<uchar>(y,sample)!=0;
        }
      }
    }
    // A single-row spot, including a two-pixel reflector, supplies no dash.
    runs.erase(std::remove_if(runs.begin(),runs.end(),[&](const PaintRun &run) {
      return run.on<min_run;
    }),runs.end());
    std::sort(runs.begin(),runs.end(),[](const PaintRun &a,const PaintRun &b) { return a.first<b.first; });
    if (on<mask.rows*.035 || last-first<mask.rows*.035) continue;
    double occupancy=double(on)/(bottom-top+1);
    int evidence_span=bottom-top+1;
    const int continuous=*std::max_element(component_support.begin(),component_support.end());
    // Solid must cover the complete near road interval, including the empty
    // portion beyond a Hough endpoint. A long isolated dash cannot define its
    // own denominator. Distinct pieces never pool into continuous evidence.
    if (near_visible>=mask.rows*.23 && double(continuous)/near_visible>=.78 &&
        double(near_on)/near_visible>=.78)
      candidate.type=LineType::Solid;
    else if (runs.size()>=3) {
      // Perspective can leave only one dash long enough for Hough. Inspect
      // the same fitted paint line for the smaller, separated far dashes.
      const auto a=runs[runs.size()-3],b=runs[runs.size()-2],c=runs.back();
      const int la=a.last-a.first+1,lb=b.last-b.first+1,lc=c.last-c.first+1;
      const int ga=b.first-a.last-1,gb=c.first-b.last-1;
      const int span=c.last-a.first+1;
      const double dash_occupancy=double(a.on+b.on+c.on)/span;
      // A yellow marking and unrelated white scenery cannot jointly supply
      // three dashes. Each identity must support the same unambiguous color.
      const auto color=[](const PaintRun &run) {
        return run.yellow*10>=run.on*7?2:(run.on-run.yellow)*10>=run.on*7?1:0;
      };
      const int paint_color=color(a);
      if (a.label && b.label && c.label && a.label!=b.label && a.label!=c.label && b.label!=c.label &&
          a.identity && b.identity && c.identity && a.identity!=b.identity &&
          a.identity!=c.identity && b.identity!=c.identity &&
          paint_color && paint_color==color(b) && paint_color==color(c) &&
          ga>=min_gap && gb>=min_gap && span>=mask.rows*.12 &&
          lb>=la*.5 && lc>=lb*.5 && gb>=ga*.5 &&
          dash_occupancy>=.22 && dash_occupancy<.78) {
        candidate.type=LineType::Dashed;occupancy=dash_occupancy;evidence_span=span;
      }
    }
    candidate.valid=true;candidate.first_y=first;
    candidate.confidence=float(std::min(1.,occupancy*(candidate.type==LineType::Unknown?.55:1.2)));
    const double support=evidence_span*occupancy*(candidate.type==LineType::Unknown?.5:1.);
    const double distance=std::abs(candidate.slope*mask.rows*.80+candidate.intercept-mask.cols*.5);
    // The nearest credible paint encloses the ego lane. A longer outer solid
    // must not win merely because it has more pixels than the dashed boundary.
    if (!best.valid || distance<best_distance-mask.cols*.025 ||
        (std::abs(distance-best_distance)<=mask.cols*.025 && support>best_support)) {
      best=candidate;best_support=support;best_distance=distance;
    }
  }
  return best;
}

void corroborate_native_white(Boundary &boundary,const Markings &coarse,const cv::Mat &native) {
  if (!boundary.valid || boundary.type!=LineType::Unknown || native.cols<=coarse.mask.cols) return;
  const int radius=std::max(2,coarse.mask.cols/240);
  const int sample_radius=cvCeil(radius*std::hypot(1.,boundary.slope));
  struct Anchor { int on=0,yellow=0; };
  std::vector<Anchor> anchor_support(coarse.identities);
  for (int y=cvRound(coarse.mask.rows*.42);y<coarse.mask.rows;++y) {
    const int x=paint_at(coarse,y,cvRound(boundary.slope*y+boundary.intercept),sample_radius);
    if (x<0) continue;
    auto &anchor=anchor_support[coarse.identity.at<int>(y,x)];
    ++anchor.on;anchor.yellow+=coarse.yellow.at<uchar>(y,x)!=0;
  }
  std::vector<int> anchors;
  for (int id=1;id<coarse.identities;++id)
    if (anchor_support[id].on>=std::max(2,coarse.mask.rows/180) &&
        (anchor_support[id].on-anchor_support[id].yellow)*10>=anchor_support[id].on*7)
      anchors.push_back(id);
  if (anchors.size()<2) return;

  // INTER_AREA pixel centers map separately in x/y. This evidence never
  // refits or moves the already selected coarse boundary.
  const double sx=double(native.cols)/coarse.mask.cols,sy=double(native.rows)/coarse.mask.rows;
  const double slope=boundary.slope*sx/sy;
  const double intercept=(boundary.intercept+.5)*sx-.5-slope*(sy*.5-.5);
  const double normal=std::hypot(1.,slope);
  // The horizontal search extent is the existing coarse fit radius mapped
  // to native pixels; division by the native normal gives its distance band.
  const double normal_band=(radius+1)*std::hypot(1.,boundary.slope)*sx/normal;
  const int y0=std::max(0,cvRound(native.rows*.60)-8),y1=std::min(native.rows,cvRound(native.rows*.90)+8);
  const int kernel=std::max(15,(native.cols/16)|1);
  const double margin=normal_band*normal+kernel/2+2;
  const double xa=slope*y0+intercept,xb=slope*(y1-1)+intercept;
  const int x0=int(std::clamp(std::floor(std::min(xa,xb)-margin),0.,double(native.cols)));
  const int x1=int(std::clamp(std::ceil(std::max(xa,xb)+margin+1),0.,double(native.cols)));
  if (x1<=x0 || y1<=y0 || int64_t(x1-x0)*(y1-y0)>160000) return;
  const cv::Rect roi(x0,y0,x1-x0,y1-y0);
  const auto paint=markings(native(roi),native.cols);
  if (paint.mask.empty()) return;

  // Establish complete rectangular identities before considering the narrow
  // hypothesis band. A path leaving the crop must never be split into votes.
  std::vector<bool> identity_edge(paint.identities,false),raw_edge(paint.raw_stats.rows,false);
  const auto edge=[&](int y,int x) {
    identity_edge[paint.identity.at<int>(y,x)]=true;
    raw_edge[paint.raw_labels.at<int>(y,x)]=true;
  };
  for (int x=0;x<roi.width;++x) { edge(0,x);edge(roi.height-1,x); }
  for (int y=0;y<roi.height;++y) { edge(y,0);edge(y,roi.width-1); }
  struct NativeRun {
    int identity=0,label=0,first=0,last=0,on=0,last_row=-1,white=0,yellow=0;
    bool raw_cut=false;
    std::vector<cv::Point2f> points;
    std::vector<int> known;
  };
  std::vector<NativeRun> groups(paint.identities);
  std::vector<bool> occupied(roi.height,false);
  for (int y=0;y<roi.height;++y) for (int x=0;x<roi.width;++x) {
    if (!paint.mask.at<uchar>(y,x) || !paint_pixel(paint,y,x)) continue;
    const int identity=paint.identity.at<int>(y,x);
    if (!identity) continue;
    auto &run=groups[identity];run.identity=identity;
    const int raw=paint.raw_labels.at<int>(y,x);
    if (narrow_component(paint.raw_stats,raw,native.cols) && raw_edge[raw]) run.raw_cut=true;
    const bool yellow=paint.yellow.at<uchar>(y,x)!=0;
    const double distance=std::abs(x+roi.x-slope*(y+roi.y)-intercept)/normal;
    if (distance<=normal_band) {
      occupied[y]=true;
      if (yellow) ++run.yellow;
      else {
        ++run.white;
        if (run.last_row!=y) {
          if (!run.on) run.first=y+roi.y;
          run.last=y+roi.y;run.last_row=y;++run.on;
        }
      }
    }
    if (yellow) continue;
    run.points.emplace_back(float(x+roi.x),float(y+roi.y));
    if (!run.label) run.label=paint.labels.at<int>(y,x);
    const int wx=cvRound((x+roi.x+.5)/sx-.5),wy=cvRound((y+roi.y+.5)/sy-.5);
    if (wx>=0 && wy>=0 && wx<coarse.mask.cols && wy<coarse.mask.rows) {
      const int known=coarse.identity.at<int>(wy,wx);
      if (known) run.known.push_back(known);
    }
  }
  std::vector<NativeRun> runs;
  for (auto &run:groups) {
    if (!run.identity || identity_edge[run.identity] || run.raw_cut ||
        run.on<std::max(2,native.rows/180) || run.white*10<(run.white+run.yellow)*7 ||
        run.points.size()<2) continue;
    cv::Vec4f direction;cv::fitLine(run.points,direction,cv::DIST_L2,0,.01,.01);
    if (std::abs(direction[0]*slope+direction[1])/normal<std::cos(CV_PI/12)) continue;
    const auto box=cv::minAreaRect(run.points);
    if ((std::max(box.size.width,box.size.height)+1)/(std::min(box.size.width,box.size.height)+1)<3) continue;
    cv::Point2f center(0,0);for (auto point:run.points) center+=point;center*=1.f/run.points.size();
    if (std::abs(center.x-slope*center.y-intercept)/normal>normal_band) continue;
    std::sort(run.known.begin(),run.known.end());
    run.known.erase(std::unique(run.known.begin(),run.known.end()),run.known.end());
    runs.push_back(std::move(run));
  }
  std::sort(runs.begin(),runs.end(),[](const NativeRun &a,const NativeRun &b) { return a.first<b.first; });
  const auto repeated=[](const NativeRun &a,const NativeRun &b) {
    for (int id:a.known) if (std::binary_search(b.known.begin(),b.known.end(),id)) return true;
    return false;
  };
  const auto dark=[&](int first,int last) {
    int count=0;for (int y=first;y<=last;++y) count+=!occupied[y-roi.y];return count;
  };
  const int min_gap=cvCeil(std::max(3,coarse.mask.rows/100)*sy);
  // Consecutive, sorted triples bound association work even on textured roads.
  for (size_t i=2;i<runs.size();++i) {
    const auto &a=runs[i-2],&b=runs[i-1],&c=runs[i];
    const int la=a.last-a.first+1,lb=b.last-b.first+1,lc=c.last-c.first+1;
    const int ga=b.first-a.last-1,gb=c.first-b.last-1,span=c.last-a.first+1;
    if (ga<min_gap || gb<min_gap || span<native.rows*.12 || lb<la*.5 || lc<lb*.5 || gb<ga*.5 ||
        a.label==b.label || a.label==c.label || b.label==c.label ||
        repeated(a,b) || repeated(a,c) || repeated(b,c)) continue;
    int matches=0;
    for (const auto *run:{&a,&b,&c}) {
      bool anchored=false;
      for (int id:anchors) if (std::binary_search(run->known.begin(),run->known.end(),id)) {
        anchored=true;break;
      }
      matches+=anchored;
    }
    const double occupancy=double(a.on+b.on+c.on)/span;
    if (matches<2 || dark(a.last+1,b.first-1)<min_gap || dark(b.last+1,c.first-1)<min_gap ||
        occupancy<.22 || occupancy>=.78) continue;
    boundary.type=LineType::Dashed;
    boundary.confidence=float(std::min(1.,occupancy*1.2));
    return;
  }
}

LaneInfo lanes(const cv::Mat &bgr,const cv::Mat &native = {}) {
  LaneInfo lane;
  const auto paint=markings(bgr);
  cv::Mat edges;
  const int top = cvRound(bgr.rows*0.42), bottom = bgr.rows-1;
  cv::Canny(paint.mask,edges,50,140);
  std::vector<cv::Vec4i> lines,left,right;
  cv::HoughLinesP(edges,lines,1,CV_PI/180,18,std::max(12,bgr.rows/32),std::max(6,bgr.rows/55));
  std::sort(lines.begin(),lines.end(),[](const cv::Vec4i &a,const cv::Vec4i &b) {
    return cv::norm(cv::Point(a[0]-a[2],a[1]-a[3]))>
           cv::norm(cv::Point(b[0]-b[2],b[1]-b[3]));
  });
  for (auto segment : lines) {
    const double dy=segment[3]-segment[1],dx=segment[2]-segment[0];
    if (std::abs(dy)<std::max(4,bgr.rows/80) || std::abs(dx/dy)>3.5) continue;
    const double slope=dx/dy, x=segment[0]+(bottom-segment[1])*slope;
    if (x > -bgr.cols*.4 && x < bgr.cols*.49 && slope < .10 &&
        std::max(segment[0],segment[2])<bgr.cols*.52 && left.size()<96) left.push_back(segment);
    if (x < bgr.cols*1.4 && x > bgr.cols*.51 && slope > -.10 &&
        std::min(segment[0],segment[2])>bgr.cols*.48 && right.size()<96) right.push_back(segment);
  }
  auto l=fit_boundary(paint,left,true),r=fit_boundary(paint,right,false);
  if (l.valid && r.valid) {
    const double near_width=(r.slope-l.slope)*bottom+r.intercept-l.intercept;
    const double ahead_width=(r.slope-l.slope)*bgr.rows*.80+r.intercept-l.intercept;
    const double crossing=(l.intercept-r.intercept)/(r.slope-l.slope);
    const double vanishing=l.slope*crossing+l.intercept;
    if (near_width<bgr.cols*.25 || near_width>bgr.cols*1.05 ||
        ahead_width<bgr.cols*.10 || ahead_width>bgr.cols*.70 ||
        crossing>bgr.rows*.68 || vanishing<bgr.cols*.25 || vanishing>bgr.cols*.75) {
      l={};r={};
    }
  }
  if (!native.empty()) {
    corroborate_native_white(l,paint,native);
    corroborate_native_white(r,paint,native);
  }
  lane.left_valid=l.valid; lane.right_valid=r.valid;
  lane.left_type=l.type; lane.right_type=r.type; lane.left_conf=l.confidence; lane.right_conf=r.confidence;
  int evidence_top=l.valid&&r.valid?std::max(l.first_y,r.first_y):
                   l.valid?l.first_y:r.valid?r.first_y:top;
  // Aligned scenery above the vanishing point must not draw a crossed
  // corridor. Clip the visible interval without changing either fitted line.
  if (l.valid && r.valid)
    evidence_top=std::max(evidence_top,cvFloor((l.intercept-r.intercept)/(r.slope-l.slope))+1);
  lane.top_y=float(evidence_top); lane.bottom_y=float(bottom);
  if (l.valid) { lane.left_x=float(l.slope*bottom+l.intercept); lane.left_top_x=float(l.slope*evidence_top+l.intercept); }
  if (r.valid) { lane.right_x=float(r.slope*bottom+r.intercept); lane.right_top_x=float(r.slope*evidence_top+r.intercept); }
  if (l.valid && r.valid && lane.right_x-lane.left_x > bgr.cols*.15)
    lane.center_offset=float((bgr.cols*.5-(lane.left_x+lane.right_x)*.5)/((lane.right_x-lane.left_x)*.5));
  return lane;
}

NavInfo navigation(const cv::Mat &bgr, const Roi &roi) {
  NavInfo result;
  if (!std::isfinite(roi.x) || !std::isfinite(roi.y) || !std::isfinite(roi.w) || !std::isfinite(roi.h) || roi.w<=0 || roi.h<=0) return result;
  const double x0=std::clamp(roi.x,0.0,1.0),y0=std::clamp(roi.y,0.0,1.0);
  const double x1=std::clamp(roi.x+roi.w,0.0,1.0),y1=std::clamp(roi.y+roi.h,0.0,1.0);
  const cv::Rect region(cvRound(x0*bgr.cols),cvRound(y0*bgr.rows),cvRound((x1-x0)*bgr.cols),cvRound((y1-y0)*bgr.rows));
  if (region.width<8 || region.height<8 || (region & cv::Rect(0,0,bgr.cols,bgr.rows)) != region) return result;
  cv::Mat hsv,blue,green,mask;
  cv::cvtColor(bgr(region),hsv,cv::COLOR_BGR2HSV);
  cv::inRange(hsv,cv::Scalar(88,95,80),cv::Scalar(135,255,255),blue);
  cv::inRange(hsv,cv::Scalar(35,95,80),cv::Scalar(88,255,255),green);
  cv::bitwise_or(blue,green,mask);
  cv::morphologyEx(mask,mask,cv::MORPH_OPEN,cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
  double best_area=0;
  for (const auto &contour: contours) {
    const double area=std::abs(cv::contourArea(contour));
    if (area<std::max(70.0,region.area()*.001) || area<best_area) continue;
    std::vector<cv::Point> poly;
    cv::approxPolyDP(contour,poly,cv::arcLength(contour,true)*.018,true);
    if (poly.size()<6 || poly.size()>10) continue;
    const double orientation=cv::contourArea(poly,true)>0?1:-1;
    std::vector<int> concave;
    for (int i=0;i<int(poly.size());++i) {
      const auto a=poly[i]-poly[(i+poly.size()-1)%poly.size()];
      const auto b=poly[(i+1)%poly.size()]-poly[i];
      if (cross(a,b)*orientation < -20) concave.push_back(i);
    }
    if (concave.size()!=2) continue;
    const cv::Point2f a=poly[concave[0]],b=poly[concave[1]],across=b-a;
    const double shaft_width=cv::norm(across);
    if (shaft_width<5) continue;
    cv::Point2f tail(0,0); int tails=0;
    for (int index: concave) {
      const cv::Point2f p=poly[index];
      for (int neighbor: {(index+int(poly.size())-1)%int(poly.size()),(index+1)%int(poly.size())}) {
        const cv::Point2f vector=cv::Point2f(poly[neighbor])-p;
        const double len=cv::norm(vector);
        if (len>shaft_width*.45 && std::abs(vector.dot(across))/(len*shaft_width)<.4) {
          tail+=cv::Point2f(poly[neighbor]); ++tails;
        }
      }
    }
    if (tails!=2) continue;
    tail*=.5f;
    const cv::Point2f center=(a+b)*.5f,direction=center-tail;
    const double length=cv::norm(direction);
    if (length<shaft_width*.55) continue;
    double head_extension=0;
    for (auto p:poly) head_extension=std::max(head_extension,double((cv::Point2f(p)-center).dot(direction))/length);
    if (head_extension<shaft_width*.35) continue;
    Direction dir=Direction::None;
    if (std::abs(direction.x)>std::abs(direction.y)*1.25) dir=direction.x<0?Direction::Left:Direction::Right;
    else if (direction.y<0 && -direction.y>std::abs(direction.x)*1.25) dir=Direction::Straight;
    if (dir==Direction::None) continue;
    const auto box=cv::boundingRect(contour);
    result.detected=true; result.dir=dir; result.confidence=.9f;
    result.box={float(box.x+region.x),float(box.y+region.y),float(box.x+box.width+region.x),float(box.y+box.height+region.y)};
    best_area=area;
  }
  return result;
}

cv::Mat road_mask(cv::Size size, const GroundCalibration &c, const std::vector<Box> &boxes) {
  cv::Mat mask(size,CV_8U,cv::Scalar(0));
  std::vector<cv::Point> polygon;
  for (auto p:c.image_points()) polygon.emplace_back(cvRound(p.x),cvRound(p.y));
  cv::fillConvexPoly(mask,polygon,cv::Scalar(255));
  cv::erode(mask,mask,cv::getStructuringElement(cv::MORPH_RECT,{9,9}));
  for (auto box:boxes) {
    if (!std::isfinite(box.left) || !std::isfinite(box.top) || !std::isfinite(box.right) || !std::isfinite(box.bottom)) continue;
    const int x0=cvRound(std::clamp(double(box.left)-8,0.0,double(size.width)));
    const int y0=cvRound(std::clamp(double(box.top)-8,0.0,double(size.height)));
    const int x1=cvRound(std::clamp(double(box.right)+8,0.0,double(size.width)));
    const int y1=cvRound(std::clamp(double(box.bottom)+8,0.0,double(size.height)));
    if (x1>x0 && y1>y0) mask(cv::Rect(x0,y0,x1-x0,y1-y0)).setTo(0);
  }
  return mask;
}

std::string calibration_key(const GroundCalibration &c) {
  std::ostringstream stream; stream.precision(12);
  stream << c.valid() << ':' << c.width_m() << ':' << c.length_m() << ':' << c.near_offset_m() << ':' << c.profile_identity();
  for (auto p:c.image_points()) stream << ':' << p.x << ':' << p.y;
  return stream.str();
}
} // namespace

void RoadGeometry::reset() {
  previous_gray_.release(); previous_mask_.release(); previous_stamp_={};
  previous_observation_={}; calibration_key_.clear(); smoothed_speed_kmh_=-1; has_observation_=false;
}

SpeedResult RoadGeometry::analyze_speed(const cv::Mat &gray, const Stamp &stamp,
                                       const GroundCalibration &c,
                                       const std::vector<Box> &boxes) {
  SpeedResult result;
  const std::string key=calibration_key(c);
  if (key!=calibration_key_) { previous_gray_.release(); previous_mask_.release(); smoothed_speed_kmh_=-1; calibration_key_=key; }
  if (stamp.static_image) { previous_gray_.release(); smoothed_speed_kmh_=-1; result.reason="静态图片不估速"; return result; }
  if (!c.is_compatible(gray.cols,gray.rows)) { previous_gray_.release(); smoothed_speed_kmh_=-1; result.reason="未标定或前视尺寸不匹配"; return result; }
  const cv::Mat native_mask=road_mask(gray.size(),c,boxes);
  cv::Mat flow_gray,mask;
  const double flow_scale=std::min(1.,384./gray.cols);
  const cv::Size flow_size(cvRound(gray.cols*flow_scale),cvRound(gray.rows*flow_scale));
  cv::resize(gray,flow_gray,flow_size,0,0,cv::INTER_AREA);
  cv::resize(native_mask,mask,flow_size,0,0,cv::INTER_NEAREST);
  const double native_x=double(gray.cols)/flow_size.width,native_y=double(gray.rows)/flow_size.height;
  const auto native=[&](cv::Point2f point) {
    return cv::Point2f(float((point.x+.5)*native_x-.5),float((point.y+.5)*native_y-.5));
  };
  // Bounded work images keep the CPU budget independent of the display size.
  // Calibration and all externally visible coordinates remain native pixels.
  const auto remember=[&] { flow_gray.copyTo(previous_gray_); mask.copyTo(previous_mask_); previous_stamp_=stamp; };
  if (previous_gray_.empty()) { remember(); result.reason="等待下一视频帧"; return result; }
  const double dt=stamp.source_s-previous_stamp_.source_s;
  if (!std::isfinite(dt) || dt<.005 || dt>.5) { remember(); smoothed_speed_kmh_=-1; result.reason="视频时间间隔无效"; return result; }
  std::vector<cv::Point2f> before,after,back;
  cv::goodFeaturesToTrack(previous_gray_,before,240,.015,5,previous_mask_,5);
  result.features=int(before.size());
  if (before.size()<12) { remember(); smoothed_speed_kmh_=-1; result.reason="路面纹理不足"; return result; }
  std::vector<uchar> forward_ok,backward_ok;
  std::vector<float> forward_error,backward_error;
  cv::calcOpticalFlowPyrLK(previous_gray_,flow_gray,before,after,forward_ok,forward_error,{15,15},2);
  cv::calcOpticalFlowPyrLK(flow_gray,previous_gray_,after,back,backward_ok,backward_error,{15,15},2);
  std::vector<cv::Point2f> metric_before,metric_after;
  for (size_t i=0;i<before.size();++i) {
    if (!forward_ok[i] || !backward_ok[i] || forward_error[i]>35 || cv::norm(before[i]-back[i])>.6 || !finite(after[i])) continue;
    const int x=cvRound(after[i].x),y=cvRound(after[i].y);
    if (x<0 || y<0 || x>=mask.cols || y>=mask.rows || !mask.at<uchar>(y,x)) continue;
    const auto a=c.map(native(before[i])),b=c.map(native(after[i]));
    if (finite(a) && finite(b)) { metric_before.push_back(a); metric_after.push_back(b); }
  }
  remember(); result.features=int(metric_before.size());
  if (metric_before.size()<12) { smoothed_speed_kmh_=-1; result.reason="可靠路面特征不足"; return result; }
  cv::Mat inliers;
  const cv::Mat transform=cv::estimateAffinePartial2D(metric_before,metric_after,inliers,cv::RANSAC,.10,2000,.995,10);
  const int count=inliers.empty()?0:cv::countNonZero(inliers);
  if (transform.empty() || count<12 || count<int(metric_before.size()*.55)) {
    smoothed_speed_kmh_=-1; result.reason="路面运动不一致"; return result;
  }
  const double a=transform.at<double>(0,0),b=transform.at<double>(1,0);
  const double scale=std::hypot(a,b),rotation=std::atan2(b,a);
  if (!cv::checkRange(transform) || std::abs(scale-1)>.035 || std::abs(rotation)>.14) {
    smoothed_speed_kmh_=-1; result.reason="路面模型缩放或旋转异常"; return result;
  }
  // Translation under rotation depends on the chosen ground origin. Without
  // the measured car-to-near-edge offset that origin is unknown, so yaw above
  // the optical-flow noise allowance (0.02 rad/s) cannot yield ego speed.
  if (!c.distance_valid() && std::abs(rotation)/dt>.02) {
    smoothed_speed_kmh_=-1; result.reason="未测车头偏移，转弯时视觉估速无效"; return result;
  }
  // With a calibrated ground plane, translation is metric even when image flow
  // varies strongly with depth. Rotation is fitted separately from translation.
  const double raw=std::hypot(transform.at<double>(0,2),transform.at<double>(1,2))/dt*3.6;
  if (!std::isfinite(raw) || raw>220) { smoothed_speed_kmh_=-1; result.reason="视觉速度超出有效范围"; return result; }
  smoothed_speed_kmh_=smoothed_speed_kmh_<0?raw:.65*smoothed_speed_kmh_+.35*raw;
  result.valid=true; result.speed_kmh=float(smoothed_speed_kmh_); result.features=count; result.reason="已标定地平面视觉估速";
  return result;
}

GeometryObservation RoadGeometry::analyze(const cv::Mat &bgr,const Stamp &stamp,
                                        const GroundCalibration &c,const Roi &roi,
                                        const std::vector<Box> &boxes) {
  const auto start=std::chrono::steady_clock::now();
  GeometryObservation result; result.stamp=stamp; result.width=bgr.cols; result.height=bgr.rows;
  if (bgr.empty() || bgr.type()!=CV_8UC3 || bgr.cols<32 || bgr.rows<32) {
    reset(); result.status=Status::Failed; result.speed.reason="前视图像无效"; return result;
  }
  if (has_observation_ && (stamp.generation!=previous_observation_.stamp.generation ||
      bgr.cols!=previous_observation_.width || bgr.rows!=previous_observation_.height)) reset();
  // A paused source can still receive ROI/calibration changes. Re-evaluate the
  // spatial outputs, while duplicate frames keep the optical-flow baseline.
  const bool duplicate=has_observation_ && stamp.sequence==previous_observation_.stamp.sequence;
  const bool reuse_speed=duplicate && calibration_key(c)==calibration_key_ &&
                        stamp.static_image==previous_observation_.stamp.static_image;
  cv::Mat gray; cv::cvtColor(bgr,gray,cv::COLOR_BGR2GRAY);
  const double road_scale=std::min(1.,480./bgr.cols);
  cv::Mat road_image;cv::resize(bgr,road_image,{},road_scale,road_scale,cv::INTER_AREA);
  result.lane=lanes(road_image,bgr); result.nav=navigation(road_image,roi);
  const float sx=float(bgr.cols)/road_image.cols,sy=float(bgr.rows)/road_image.rows;
  auto &lane=result.lane;
  lane.left_x*=sx;lane.right_x*=sx;lane.left_top_x*=sx;lane.right_top_x*=sx;
  lane.top_y*=sy;lane.bottom_y*=sy;
  auto &box=result.nav.box;box.left*=sx;box.right*=sx;box.top*=sy;box.bottom*=sy;
  result.speed=reuse_speed?previous_observation_.speed:analyze_speed(gray,stamp,c,boxes);
  result.status=Status::Valid;
  result.process_ms=float(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
  previous_observation_=result; has_observation_=true;
  return result;
}
} // namespace adas
