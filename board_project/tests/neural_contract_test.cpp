#include "inference/neural_contract.h"
#include <QCoreApplication>
#include <QTemporaryDir>
#include <QFile>
#include <QCryptographicHash>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <cassert>
#include <limits>
#include <iostream>
#include <cmath>
#include <algorithm>

adas::LaneTensorData road(bool gaps) {
  adas::LaneTensorData d;
  d.x.resize(384); d.present.assign(384,.99f);
  d.paint.assign(1152,-12); d.type.assign(12,-12);
  for(int c=0;c<4;++c) {
    d.type[c*3+(gaps?2:1)]=12;
    for(int r=0;r<96;++r) {
      d.x[c*96+r]=.05f+.3f*c;
      d.paint[(c*96+r)*3+((!gaps||r%24<12)?1:2)]=12;
    }
  }
  return d;
}
void eight_class_rk_sigmoid_head() {
  adas::NeuralModelSpec spec;spec.output="yolov5_rk_sigmoid_nchw_v1";
  spec.classes={"person","vehicle","crosswalk","light_red","light_green","cone","barrier","debris"};
  for(auto &anchors:spec.anchors)anchors={10,13,16,30,33,23};
  std::vector<std::vector<float>> heads;
  for(int stride:{8,16,32})heads.emplace_back(3*13*(640/stride)*(640/stride),0.f);
  std::vector<adas::NeuralDetection> out;std::string error;
  assert(adas::decode_neural_scene(heads,spec,out,error)&&out.empty());
  const int grid=80*80;
  for(int c=0;c<4;++c)heads[0][c*grid]=.5f;
  heads[0][4*grid]=.3f;heads[0][8*grid]=.3f;
  assert(adas::decode_neural_scene(heads,spec,out,error)&&out.empty()); // .09, never sigmoid twice.
  heads[0][4*grid]=.9f;heads[0][8*grid]=.9f;
  assert(adas::decode_neural_scene(heads,spec,out,error)&&out.size()==1);
  assert(out[0].class_id==3&&std::abs(out[0].confidence-.81f)<1e-5f);
  // Identical overlapping candidates in both export contracts must have the
  // same boxes, product score and class-aware NMS result.
  for(auto &head:heads)std::fill(head.begin(),head.end(),.001f);
  for(int a=0;a<2;++a) {
    for(int c=0;c<4;++c)heads[0][(a*13+c)*grid]=.5f;
    heads[0][(a*13+4)*grid]=a==0?.9f:.8f;
    heads[0][(a*13+8)*grid]=.9f;
  }
  spec.anchors[0]={10,13,10,13,33,23};
  assert(adas::decode_neural_scene(heads,spec,out,error)&&out.size()==1);
  auto logits=heads;
  for(auto &head:logits)for(auto &value:head)value=std::log(value/(1-value));
  auto raw_spec=spec;raw_spec.output="yolov5_raw_nchw_v1";
  std::vector<adas::NeuralDetection> raw;
  assert(adas::decode_neural_scene(logits,raw_spec,raw,error)&&raw.size()==out.size());
  assert(raw[0].class_id==out[0].class_id&&std::abs(raw[0].confidence-out[0].confidence)<1e-5f);
  assert(std::abs(raw[0].box.left-out[0].box.left)<1e-5f&&std::abs(raw[0].box.right-out[0].box.right)<1e-5f);
  assert(std::abs(raw[0].box.top-out[0].box.top)<1e-5f&&std::abs(raw[0].box.bottom-out[0].box.bottom)<1e-5f);
  heads[0][0]=1.1f;assert(!adas::decode_neural_scene(heads,spec,out,error)&&out.empty());
  heads[0][0]=.5f;spec.output="unknown";assert(!adas::decode_neural_scene(heads,spec,out,error));
  std::vector<adas::NeuralTensorSpec> shape{{"p3",{1,39,80,80}},{"p4",{1,39,40,40}},{"p5",{1,39,20,20}}};
  assert(adas::validate_neural_outputs(shape,true,8,error));
  shape[0].shape[1]=42;assert(!adas::validate_neural_outputs(shape,true,8,error));
}
int main(int argc,char **argv) {
  QCoreApplication app(argc,argv);
  eight_class_rk_sigmoid_head();
  adas::LaneInfo lane;std::string error;
  auto d=road(true);
  assert(adas::decode_neural_lane(d,{0,.42,1,.58},1280,720,lane,error));
  assert(lane.left_valid&&lane.right_valid);
  assert(lane.left_type==adas::LineType::Dashed&&lane.right_type==adas::LineType::Dashed);
  assert(lane.left_x>400&&lane.left_x<500&&lane.right_x>800&&lane.right_x<880);
  // Unknown/occluded intervals cannot count as observed paint gaps.
  for(int r=0;r<96;++r)if(r%24>=12)for(int c=0;c<4;++c) {
    d.paint[(c*96+r)*3+2]=-12;d.paint[(c*96+r)*3]=12;
  }
  assert(adas::decode_neural_lane(d,{0,.42,1,.58},1280,720,lane,error));
  assert(lane.right_type==adas::LineType::Unknown);
  d=road(false);assert(adas::decode_neural_lane(d,{0,.42,1,.58},1280,720,lane,error));
  assert(lane.left_type==adas::LineType::Solid);
  // A farther boundary cannot substitute for the missing ego boundary.
  d=road(true);for(int r=0;r<96;++r)d.present[2*96+r]=0;
  assert(adas::decode_neural_lane(d,{0,.42,1,.58},1280,720,lane,error));
  assert(!lane.right_valid);
  d=road(true);d.x[0]=std::numeric_limits<float>::quiet_NaN();
  assert(!adas::decode_neural_lane(d,{0,.42,1,.58},1280,720,lane,error));
  d=road(true);d.paint.pop_back();
  assert(!adas::decode_neural_lane(d,{0,.42,1,.58},1280,720,lane,error));
  // ROI may begin above the horizon. Never extrapolate visible boundaries into unsupported rows.
  d=road(true);
  for(int r=0;r<96;++r){d.x[96+r]=.55f-.25f*r/95;d.x[192+r]=.45f+.25f*r/95;if(r<20){d.present[96+r]=d.present[192+r]=0;}}
  assert(adas::decode_neural_lane(d,{0,.42,1,.58},1280,720,lane,error));
  assert(lane.left_valid&&lane.right_valid&&lane.right_type==adas::LineType::Dashed);
  assert(lane.top_y>380&&lane.right_top_x>lane.left_top_x);
  QTemporaryDir dir;adas::NeuralBundleSpec bundle;QString message;
  assert(!adas::read_neural_manifest(dir.filePath("missing.json"),bundle,message));
  assert(!message.isEmpty());
  QFile f(dir.filePath("manifest.json"));assert(f.open(QIODevice::WriteOnly));
  f.write("{\"schema\":1,\"target\":\"rk3588\"}");f.close();
  assert(!adas::read_neural_manifest(f.fileName(),bundle,message));

  const QByteArray bytes("synthetic contract fixture, not a model");
  for(const char *name:{"scene.rknn","lane.rknn"}) {
    QFile weights(dir.filePath(name));assert(weights.open(QIODevice::WriteOnly));weights.write(bytes);
  }
  const auto hash=QString::fromLatin1(QCryptographicHash::hash(bytes,QCryptographicHash::Sha256).toHex());
  QJsonArray classes;for(const char *name:{"person","vehicle","crosswalk","light_red","light_yellow","light_green","cone","barrier","debris","nav_left","nav_straight","nav_right"})classes.append(name);
  QJsonObject input{{"width",640},{"height",640},{"layout","NHWC"},{"format","RGB8"},{"mean",QJsonArray{0,0,0}},{"std",QJsonArray{255,255,255}}};
  QJsonObject scene{{"file","scene.rknn"},{"sha256",hash},{"input",input},{"output","yolov5_raw_nchw_v1"},{"quantization","int8"},{"runtime_api_prefix","fixture"},{"classes",classes},
    {"anchors",QJsonArray{QJsonArray{10,13,16,30,33,23},QJsonArray{30,61,62,45,59,119},QJsonArray{116,90,156,198,373,326}}}};
  input["width"]=512;input["height"]=256;
  QJsonObject road_model{{"file","lane.rknn"},{"sha256",hash},{"input",input},{"output","lane_rows_4x96_v1"},{"quantization","int8"},{"runtime_api_prefix","fixture"}};
  QJsonObject manifest{{"schema",1},{"target","rk3568"},{"version","fixture"},{"toolkit_version","fixture"},{"road_roi",QJsonArray{0,.42,1,.58}},{"scene",scene},{"lane",road_model}};
  auto write=[&] {assert(f.open(QIODevice::WriteOnly|QIODevice::Truncate));f.write(QJsonDocument(manifest).toJson());f.close();};
  write();assert(adas::read_neural_manifest(f.fileName(),bundle,message));
  assert(bundle.scene.classes.size()==12&&bundle.lane.width==512);
  const auto full_manifest=manifest;
  QJsonArray eight;for(const char *name:{"person","vehicle","crosswalk","light_red","light_green","cone","barrier","debris"})eight.append(name);
  scene["classes"]=eight;scene["output"]="yolov5_rk_sigmoid_nchw_v1";
  manifest["schema"]=2;manifest["scene"]=scene;manifest["lane"]=QJsonValue(QJsonValue::Null);write();
  assert(adas::read_neural_manifest(f.fileName(),bundle,message));
  assert(bundle.scene.classes.size()==8&&bundle.lane.path.isEmpty());
  manifest.remove("lane");write();assert(!adas::read_neural_manifest(f.fileName(),bundle,message));
  manifest["lane"]=false;write();assert(!adas::read_neural_manifest(f.fileName(),bundle,message));
  manifest["lane"]=QJsonValue(QJsonValue::Null);manifest["schema"]=1;write();
  assert(!adas::read_neural_manifest(f.fileName(),bundle,message));
  manifest["schema"]=2;
  eight.removeLast();scene["classes"]=eight;manifest["scene"]=scene;write();
  assert(!adas::read_neural_manifest(f.fileName(),bundle,message)); // No debris capability.
  manifest=full_manifest;scene=manifest["scene"].toObject();

  scene["sha256"]=QString(64,'0');manifest["scene"]=scene;write();
  assert(!adas::read_neural_manifest(f.fileName(),bundle,message));
  scene["sha256"]=hash;classes.removeLast();scene["classes"]=classes;manifest["scene"]=scene;write();
  assert(adas::read_neural_manifest(f.fileName(),bundle,message)); // Visual navigation is optional.
  classes.removeAt(8);scene["classes"]=classes;manifest["scene"]=scene;write();
  assert(!adas::read_neural_manifest(f.fileName(),bundle,message));
  scene["file"]="../scene.rknn";manifest["scene"]=scene;write();
  assert(!adas::read_neural_manifest(f.fileName(),bundle,message));

  std::vector<adas::NeuralTensorSpec> attrs{{"lane_x",{1,4,96}},{"lane_present",{1,4,96}},{"lane_paint",{1,4,96,3}},{"lane_type",{1,4,3}}};
  assert(adas::validate_neural_outputs(attrs,false,0,error));
  std::swap(attrs[0],attrs[1]);assert(!adas::validate_neural_outputs(attrs,false,0,error));
  std::swap(attrs[0],attrs[1]);attrs[0].shape={1,96,4};assert(!adas::validate_neural_outputs(attrs,false,0,error));
  adas::NeuralModelSpec scene_spec;scene_spec.output="yolov5_raw_nchw_v1";scene_spec.classes.resize(12);
  for(auto &head:scene_spec.anchors)head={10,13,16,30,33,23};
  std::vector<std::vector<float>> heads;
  for(int stride:{8,16,32})heads.emplace_back(3*17*(640/stride)*(640/stride),-20);
  std::vector<adas::NeuralDetection> detected;
  assert(adas::decode_neural_scene(heads,scene_spec,detected,error)&&detected.empty());
  const int grid=80*80;heads[0][4*grid]=std::log(.3f/.7f);heads[0][(5+5)*grid]=std::log(.3f/.7f);
  assert(adas::decode_neural_scene(heads,scene_spec,detected,error)&&detected.empty()); // .09 is never light evidence.
  heads[0][4*grid]=12;heads[0][(5+5)*grid]=12;
  assert(adas::decode_neural_scene(heads,scene_spec,detected,error)&&detected.size()==1&&detected[0].class_id==5);
  for(auto &head:heads)std::fill(head.begin(),head.end(),0);
  assert(!adas::decode_neural_scene(heads,scene_spec,detected,error)&&detected.empty()); // Dense logits fail before NMS.
  std::cout<<"neural contracts passed\n";
}
