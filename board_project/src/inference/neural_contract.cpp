#include "inference/neural_contract.h"
#include <QCryptographicHash>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QRegularExpression>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace adas {
namespace {
void require(bool value,const char *message) {if(!value)throw std::runtime_error(message);}
bool finite(const std::vector<float> &values) {
  return std::all_of(values.begin(),values.end(),[](float v){return std::isfinite(v);});
}
std::pair<int,float> choice(const float *v) {
  const int i=int(std::max_element(v,v+3)-v);
  float sum=0;for(int j=0;j<3;++j)sum+=std::exp(v[j]-v[i]);
  return {i,1/sum};
}
NeuralModelSpec read_model(const QJsonObject &obj,const QDir &base,bool scene) {
  NeuralModelSpec s;
  const QString filename=obj.value("file").toString();
  require(!filename.isEmpty()&&!QDir::isAbsolutePath(filename)&&!filename.contains(".."),"model file must be relative to bundle directory");
  s.path=QFileInfo(base.filePath(filename)).canonicalFilePath();
  require(!s.path.isEmpty()&&s.path.startsWith(base.canonicalPath()+"/"),"model file missing or outside bundle directory");
  s.sha256=obj.value("sha256").toString().toLower();
  require(QRegularExpression("^[0-9a-f]{64}$").match(s.sha256).hasMatch(),"invalid model SHA256");
  QFile file(s.path);require(file.open(QIODevice::ReadOnly)&&file.size()>0&&file.size()<=512LL*1024*1024,"cannot read bounded model file");
  QCryptographicHash hash(QCryptographicHash::Sha256);require(hash.addData(&file),"cannot hash model");
  require(QString::fromLatin1(hash.result().toHex())==s.sha256,"model SHA256 mismatch");
  const auto input=obj.value("input").toObject();
  require(input.value("layout").toString()=="NHWC"&&input.value("format").toString()=="RGB8", "model input must be NHWC RGB8");
  const auto mean=input.value("mean").toArray(),stddev=input.value("std").toArray();
  require(mean.size()==3&&stddev.size()==3,"input normalization is required");
  for(int i=0;i<3;++i)require(mean[i].toDouble(-1)==0&&stddev[i].toDouble(-1)==255,"input normalization must match exporter: mean0/std255");
  s.width=input.value("width").toInt();s.height=input.value("height").toInt();
  require(s.width==(scene?640:512)&&s.height==(scene?640:256),"unsupported model input dimensions");
  require(obj.value("quantization").toString()=="int8","only INT8 ADAS models are accepted");
  s.output=obj.value("output").toString();
  require(scene?(s.output=="yolov5_raw_nchw_v1"||s.output=="yolov5_rk_sigmoid_nchw_v1"):s.output=="lane_rows_4x96_v1","unsupported model output contract");
  s.runtime_api_prefix=obj.value("runtime_api_prefix").toString();
  require(!s.runtime_api_prefix.isEmpty(),"runtime_api_prefix is required");
  if(scene) {
    std::set<std::string> names;
    for(const auto &value:obj.value("classes").toArray()) {
      auto name=value.toString().toStdString();require(!name.empty()&&names.insert(name).second,"empty/duplicate scene class");s.classes.push_back(name);
    }
    require(s.classes.size()>=8&&s.classes.size()<=64,"unsupported class count");
    for(const char *name:{"person","vehicle","crosswalk","light_red","light_green","cone","barrier","debris"})
      require(names.count(name),"scene model lacks a mandatory ADAS class");
    const auto anchors=obj.value("anchors").toArray();require(anchors.size()==3,"three anchor heads required");
    for(int h=0;h<3;++h) {
      auto row=anchors[h].toArray();require(row.size()==6,"six anchor dimensions required per head");
      for(int a=0;a<6;++a) {s.anchors[h][a]=float(row[a].toDouble());require(std::isfinite(s.anchors[h][a])&&s.anchors[h][a]>0&&s.anchors[h][a]<=1280,"invalid anchor dimension");}
    }
  }
  return s;
}
}
bool read_neural_manifest(const QString &path,NeuralBundleSpec &spec,QString &error) {
  spec={};error.clear();
  try {
    QFile file(path);require(file.open(QIODevice::ReadOnly),"ADAS model bundle is not configured: manifest missing");
    require(file.size()<=65536,"manifest too large");QJsonParseError parse;
    const auto document=QJsonDocument::fromJson(file.readAll(),&parse);
    require(parse.error==QJsonParseError::NoError&&document.isObject(),"invalid manifest JSON");
    const auto obj=document.object();const int schema=obj.value("schema").toInt();
    require((schema==1||schema==2)&&obj.value("target").toString()=="rk3568","manifest schema/target mismatch");
    NeuralBundleSpec result;result.version=obj.value("version").toString();require(!result.version.isEmpty(),"bundle version is required");
    require(!obj.value("toolkit_version").toString().isEmpty(),"toolkit version is required");
    const auto roi=obj.value("road_roi").toArray();require(roi.size()==4,"road ROI is required");
    result.road_roi={roi[0].toDouble(-1),roi[1].toDouble(-1),roi[2].toDouble(-1),roi[3].toDouble(-1)};
    auto r=result.road_roi;require(std::isfinite(r.x)&&std::isfinite(r.y)&&std::isfinite(r.w)&&std::isfinite(r.h)&&r.x>=0&&r.y>=0&&r.w>0&&r.h>0&&r.x+r.w<=1&&r.y+r.h<=1,"invalid road ROI");
    const auto base=QFileInfo(path).absoluteDir();result.scene=read_model(obj.value("scene").toObject(),base,true);
    const auto lane=obj.value("lane");
    if(lane.isObject())result.lane=read_model(lane.toObject(),base,false);
    else require(schema==2&&lane.isNull(),"lane must be configured or explicitly null in schema2");
    spec=std::move(result);return true;
  } catch(const std::exception &e) {error=QString::fromUtf8(e.what());return false;}
}
bool decode_neural_lane(const LaneTensorData &d,const Roi &roi,int width,int height,LaneInfo &lane,std::string &error) {
  lane={};error.clear();
  if(width<=0||height<=0||d.x.size()!=384||d.present.size()!=384||d.paint.size()!=1152||d.type.size()!=12||
      !finite(d.x)||!finite(d.present)||!finite(d.paint)||!finite(d.type)||
      std::any_of(d.x.begin(),d.x.end(),[](float v){return v<0||v>1;})||
      std::any_of(d.present.begin(),d.present.end(),[](float v){return v<0||v>1;})||
      !std::isfinite(roi.x)||!std::isfinite(roi.y)||!std::isfinite(roi.w)||!std::isfinite(roi.h)||
      roi.x<0||roi.y<0||roi.w<=0||roi.h<=0||roi.x+roi.w>1||roi.y+roi.h>1) {
    error="lane tensor shape, finite values or ROI invalid";return false;
  }
  struct Boundary {bool valid=false;int first=96,last=-1;double intercept=0,slope=0;float top=0,bottom=0,confidence=0;LineType type=LineType::Unknown;};
  auto boundary=[&](int c) {
    Boundary b;double sy=0,sx=0,syy=0,syx=0;int n=0,paints=0,gaps=0,longest=0,run=0,segments=0,gap_segments=0;
    int previous=-1,run_length=0;float confidence=0;
    auto finish=[&] {if(run_length>=3){if(previous==1)++segments;else if(previous==2)++gap_segments;}run_length=0;};
    for(int r=0;r<96;++r) {
      const int i=c*96+r;const auto paint=choice(&d.paint[i*3]);
      const bool present=d.present[i]>=.8f&&d.present[i]<=1&&d.x[i]>=0&&d.x[i]<=1;
      const double y=double(r)/95;
      if(present) {sy+=y;sx+=d.x[i];syy+=y*y;syx+=y*d.x[i];++n;confidence+=d.present[i];b.first=std::min(b.first,r);b.last=std::max(b.last,r);}
      const int state=present&&paint.second>=.8f?paint.first:0;
      if(state!=previous){finish();previous=state;}
      ++run_length;
      if(state==1){++paints;longest=std::max(longest,++run);}else {run=0;if(state==2)++gaps;}
      // Unknown intervals break evidence chains; they are never paint gaps.
      if(state==0){segments=0;gap_segments=0;}
    }
    finish();const auto type=choice(&d.type[c*3]);
    const double determinant=n*syy-sy*sy;
    if(n<64||paints<12||determinant<1e-6||b.last<90)return b;
    const double slope=(n*syx-sy*sx)/determinant,intercept=(sx-slope*sy)/n;
    float residual=0;
    for(int r=0;r<96;++r)if(d.present[c*96+r]>=.8f)
      residual=std::max(residual,float(std::abs(intercept+slope*r/95-d.x[c*96+r])));
    b.intercept=intercept;b.slope=slope;
    b.top=float(roi.x+roi.w*(intercept+slope*b.first/95));b.bottom=float(roi.x+roi.w*(intercept+slope*b.last/95));
    if(residual>.05f||b.bottom<0||b.bottom>1||b.top<0||b.top>1)return Boundary{};
    // Slots are supervised ego-boundary identities. Outer slots never fill a missing ego slot.
    if((c==1&&(b.bottom<.10f||b.bottom>=.49f))||(c==2&&(b.bottom<=.51f||b.bottom>.90f)))return Boundary{};
    b.valid=true;b.confidence=confidence/n;
    if(type.second>=.85f&&type.first==2&&segments>=3&&gap_segments>=2&&paints>=24&&gaps>=6)b.type=LineType::Dashed;
    if(type.second>=.85f&&type.first==1&&longest>=58&&paints>=82&&gaps<=6)b.type=LineType::Solid;
    return b;
  };
  auto left=boundary(1),right=boundary(2);
  int first=left.valid?left.first:right.valid?right.first:0;
  int last=left.valid?left.last:right.valid?right.last:95;
  if(left.valid&&right.valid) {
    first=std::max(left.first,right.first);last=std::min(left.last,right.last);
    auto limit=[&](Boundary &b){b.top=float(roi.x+roi.w*(b.intercept+b.slope*first/95));b.bottom=float(roi.x+roi.w*(b.intercept+b.slope*last/95));};
    limit(left);limit(right);
    const float span=right.bottom-left.bottom;
    if(last-first<32||span<.18f||span>.85f||right.top<=left.top){left={};right={};}
  }
  lane.left_valid=left.valid;lane.right_valid=right.valid;
  lane.left_type=left.type;lane.right_type=right.type;
  lane.left_x=left.bottom*width;lane.right_x=right.bottom*width;
  lane.left_top_x=left.top*width;lane.right_top_x=right.top*width;
  lane.left_conf=left.confidence;lane.right_conf=right.confidence;
  lane.top_y=float((roi.y+roi.h*first/95)*height);lane.bottom_y=float((roi.y+roi.h*last/95)*height);
  if(left.valid&&right.valid)lane.center_offset=(.5f-(left.bottom+right.bottom)/2)/((right.bottom-left.bottom)/2);
  return true;
}
}
namespace adas {
bool validate_neural_outputs(const std::vector<NeuralTensorSpec>&t,bool scene,size_t classes,std::string&error) {
  error.clear();if(t.size()!=size_t(scene?3:4)){error="output count mismatch";return false;}
  const char*names[]={"lane_x","lane_present","lane_paint","lane_type"};
  const std::vector<std::vector<unsigned>> shapes{{1,4,96},{1,4,96},{1,4,96,3},{1,4,3}};
  for(size_t i=0;i<t.size();++i){
    auto name=t[i].name;if(name.endsWith(":0"))name.chop(2);
    if(scene){unsigned side=640/(8u<<i);if(t[i].shape!=std::vector<unsigned>{1,unsigned(3*(classes+5)),side,side}){error="scene head shape/order mismatch";return false;}}
    else if(name!=names[i]||t[i].shape!=shapes[i]){error="lane tensor name/shape/order mismatch";return false;}
  }
  return true;
}
bool decode_neural_scene(const std::vector<std::vector<float>>&heads,const NeuralModelSpec&spec,std::vector<NeuralDetection>&out,std::string&error) {
  out.clear();error.clear();std::vector<NeuralDetection> candidates;
  auto fail=[&](const char*message){error=message;return false;};
  const auto classes=spec.classes.size();if(heads.size()!=3||classes<8||classes>64)return fail("scene tensor count/classes invalid");
  const bool probabilities=spec.output=="yolov5_rk_sigmoid_nchw_v1";
  if(!probabilities&&spec.output!="yolov5_raw_nchw_v1")return fail("scene output contract invalid");
  const auto sigmoid=[](float x){return x>=0?1.f/(1.f+std::exp(-x)):std::exp(x)/(1.f+std::exp(x));};
  const auto activate=[&](float x){return probabilities?x:sigmoid(x);};
  for(size_t h=0;h<3;++h){
    const unsigned stride=8u<<h,side=640/stride,grid=side*side,props=unsigned(classes+5);
    const auto &head=heads[h];if(head.size()!=3*props*grid||!finite(head))return fail("scene tensor shape/values invalid");
    if(probabilities&&std::any_of(head.begin(),head.end(),[](float x){return x<0||x>1;}))return fail("RK sigmoid output outside probability range");
    for(unsigned a=0;a<3;++a)for(unsigned cell=0;cell<grid;++cell){
      const unsigned base=a*props*grid+cell;const float obj=activate(head[base+4*grid]);if(obj<.25f)continue;
      int best=0;float raw=head[base+5*grid];
      for(unsigned c=1;c<classes;++c)if(head[base+(5+c)*grid]>raw){raw=head[base+(5+c)*grid];best=int(c);}
      const float score=obj*activate(raw);if(score<.25f)continue;
      if(candidates.size()>=1024)return fail("scene candidate limit exceeded before NMS");
      const float x=(activate(head[base])*2-.5f+cell%side)*stride;
      const float y=(activate(head[base+grid])*2-.5f+cell/side)*stride;
      const float sw=activate(head[base+2*grid])*2,sh=activate(head[base+3*grid])*2;
      const float w=sw*sw*spec.anchors[h][a*2],hh=sh*sh*spec.anchors[h][a*2+1];
      if(!std::isfinite(w)||!std::isfinite(hh)||w<=0||hh<=0)return fail("scene anchor/box invalid");
      candidates.push_back({best,{x-w/2,y-hh/2,x+w/2,y+hh/2},score});
    }
  }
  std::sort(candidates.begin(),candidates.end(),[](const auto&a,const auto&b){return a.confidence>b.confidence;});
  std::vector<bool> suppressed(candidates.size());
  for(size_t i=0;i<candidates.size();++i){
    if(suppressed[i])continue;
    if(out.size()>=300){out.clear();return fail("scene output limit exceeded");}
    out.push_back(candidates[i]);const auto&a=candidates[i].box;
    for(size_t j=i+1;j<candidates.size();++j){
      if(suppressed[j]||candidates[j].class_id!=candidates[i].class_id)continue;
      const auto&b=candidates[j].box;
      const float overlap=std::max(0.f,std::min(a.right,b.right)-std::max(a.left,b.left))*std::max(0.f,std::min(a.bottom,b.bottom)-std::max(a.top,b.top));
      const float total=(a.right-a.left)*(a.bottom-a.top)+(b.right-b.left)*(b.bottom-b.top)-overlap;
      if(total>0&&overlap/total>.45f)suppressed[j]=true;
    }
  }
  return true;
}
}
