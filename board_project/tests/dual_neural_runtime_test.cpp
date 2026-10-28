// Production runtime with SDK/RGA boundary doubles, never a model-accuracy test.
#include "inference/neural_models.h"
#include "inference/rga_preprocessor.h"
#include <rknn_api.h>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cstring>
#include <map>
#include <iostream>
#include <stdexcept>
namespace {
void expect(bool b,const char *s){if(!b)throw std::runtime_error(s);}
const std::vector<std::string> traffic_names{
    "crosswalk", "person", "light_red", "light_green", "light_black", "light_yellow"};
const std::vector<std::string> coco_names{
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
    "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
    "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet",
    "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"};
const std::array<std::array<float,6>,3> traffic_anchors{{
    {5.64062f,12.39062f,7.85547f,18.5f,10.71875f,28.14062f},
    {63.5f,9.07812f,15.92188f,41.3125f,25.48438f,69.9375f},
    {218.375f,16.32812f,49.90625f,100.3125f,612.5f,90.f}}};

QJsonArray names_json(const std::vector<std::string> &names) {
  QJsonArray result;
  for (const auto &name : names) result.append(QString::fromStdString(name));
  return result;
}

struct Fixture {
  QTemporaryDir directory;
  QJsonObject manifest;
  adas::NeuralBundleSpec bundle;
  QString error;

  Fixture() {
    expect(directory.isValid(), "temporary directory must be created");
    const auto traffic = model("traffic.rknn", traffic_names, "yolov5_raw_nchw_v1");
    const auto vehicle = model("vehicle.rknn", coco_names, "yolov5_rk_sigmoid_nchw_v1");
    manifest = {{"schema", 3}, {"target", "rk3568"}, {"version", "dual640-test"},
        {"toolkit_version", "2.3.2"}, {"traffic", traffic}, {"vehicle", vehicle}};
  }

  QJsonObject model(const char *filename, const std::vector<std::string> &names, const char *output) {
    const QByteArray bytes = QByteArray("manifest test fixture, not an RKNN model: ") + filename;
    QFile file(directory.filePath(filename));
    expect(file.open(QIODevice::WriteOnly), "fixture model must be writable");
    expect(file.write(bytes) == bytes.size(), "fixture model bytes must be written");
    const auto hash = QString::fromLatin1(QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
    QJsonArray anchors;
    const auto dimensions = names == traffic_names ? traffic_anchors :
        std::array<std::array<float,6>,3>{{{10,13,16,30,33,23}, {30,61,62,45,59,119}, {116,90,156,198,373,326}}};
    for (const auto &head : dimensions) {
      QJsonArray values;
      for (const float value : head) values.append(double(value));
      anchors.append(values);
    }
    return {{"file", filename}, {"sha256", hash}, {"quantization", "int8"}, {"runtime_api_prefix", "2.3.2 ("},
        {"input", QJsonObject{{"width", 640}, {"height", 640}, {"layout", "NHWC"}, {"format", "RGB8"},
            {"mean", QJsonArray{0, 0, 0}}, {"std", QJsonArray{255, 255, 255}}}},
        {"output", output}, {"classes", names_json(names)},
        {"anchors", anchors}};
  }

  bool read() {
    QFile file(directory.filePath("manifest.json"));
    expect(file.open(QIODevice::WriteOnly | QIODevice::Truncate), "manifest must be writable");
    file.write(QJsonDocument(manifest).toJson());
    file.close();
    return adas::read_neural_manifest(file.fileName(), bundle, error);
  }
};


struct Context { bool traffic=false; std::array<std::vector<float>,3> heads; };
std::map<rknn_context,Context> contexts;
rknn_context next_context=1;
bool fail_vehicle=false,fail_traffic=false,invalid_vehicle_head=false;
unsigned calls=0,releases=0,last_pad=255;
std::vector<bool> role_runs;
adas::FramePacket packet(adas::Layout layout=adas::Layout::Single){
  adas::FramePacket p;p.width=1280;p.height=720;p.stride=2560+32;p.layout=layout;
  p.pixels=std::make_shared<adas::FrameStorage>(size_t(p.stride)*p.height);
  p.stamp={1,55,12,12,false};return p;
}
void test_roles(){
 Fixture f;expect(f.read(),"fixture manifest invalid");
 qputenv("ADAS_MODEL_MANIFEST",f.directory.filePath("manifest.json").toUtf8());
 adas::NeuralModels m;expect(m.initialize(f.directory.path()),"dual initialization failed");
 expect(contexts.size()==2,"two resident model roles were not loaded");
 const auto p=packet();std::array<adas::Roi,3> lights{{{0,0,1,1},{0,0,1,1},{0,0,1,1}}};
 auto v=m.scene(p,adas::ViewId::Front,lights);
 expect(v.role==adas::SceneModelRole::Vehicle,"default inference role must be Vehicle");
 expect(v.vision.model_ready&&calls==1&&last_pad==0,"vehicle did not execute black-padded RGA path");
 const auto &vo=v.vision.observation;
 expect(vo.components==adas::ObjectObservation::Combined&&vo.traffic_status==adas::Status::Valid&&vo.vehicle_status==adas::Status::Valid&&vo.roadblock_status==adas::Status::Missing&&vo.signal_status==adas::Status::Missing,"vehicle capability isolation failed");
 expect(vo.obstacles.size()==1&&vo.obstacles[0].class_id==1&&!vo.signal_observation,"COCO person routing failed");
 expect(v.input_mapping.source_crop.w==1280&&v.input_mapping.content.y==140&&v.input_mapping.content.h==360,"single HDMI became a mosaic crop");
 auto t=m.scene(p,adas::ViewId::Front,lights,{0,0,1,1},adas::SceneModelRole::Traffic);
 const auto &to=t.vision.observation;
 expect(t.vision.model_ready&&calls==2&&last_pad==0,"traffic did not execute second context");
 expect(to.components==(adas::ObjectObservation::Crosswalk|adas::ObjectObservation::Signal)&&to.crosswalk_status==adas::Status::Valid&&to.signal_status==adas::Status::Valid&&to.traffic_status==adas::Status::Missing&&to.vehicle_status==adas::Status::Missing,"traffic source refreshed primary capabilities");
 expect(to.obstacles.empty()&&t.vision.overlays.size()==1&&t.vision.overlays[0].class_id==12,"traffic person must be supplementary, never primary obstacles");
 expect(role_runs==std::vector<bool>({false,true})&&releases==2,"contexts or output release order wrong");
 auto side=m.scene(packet(adas::Layout::Mosaic),adas::ViewId::Left,lights,{0,0,1,1},adas::SceneModelRole::Traffic);
 expect(side.vision.model_ready&&side.vision.observation.components==adas::ObjectObservation::Crosswalk&&side.vision.observation.signal_status==adas::Status::Missing,"side lamps must not own front signal authority");
 expect(side.input_mapping.source_crop.x==640&&side.input_mapping.source_crop.y==360&&side.vision.observation.width==640,"mosaic view offset or local coordinates lost");
 fail_vehicle=true;
 auto bad=m.scene(p,adas::ViewId::Front,lights,{0,0,1,1},adas::SceneModelRole::Vehicle);
 expect(!bad.vision.model_ready&&bad.vision.observation.traffic_status==adas::Status::Failed&&bad.vision.observation.crosswalk_status==adas::Status::Missing,"vehicle failure leaked into traffic clock");
 auto good=m.scene(p,adas::ViewId::Front,lights,{0,0,1,1},adas::SceneModelRole::Traffic);
 expect(good.vision.model_ready&&good.vision.observation.crosswalk_status==adas::Status::Valid,"one role runtime failure disabled its peer");
 fail_vehicle=false;fail_traffic=true;
 bad=m.scene(p,adas::ViewId::Front,lights,{0,0,1,1},adas::SceneModelRole::Traffic);
 expect(!bad.vision.model_ready&&bad.vision.observation.crosswalk_status==adas::Status::Failed&&bad.vision.observation.traffic_status==adas::Status::Missing,"traffic failure forged primary failure/empty");
 fail_traffic=false;
 qputenv("ADAS_MODEL_MANIFEST","/nonexistent-dual-test/manifest.json");
 expect(!m.initialize(f.directory.path())&&contexts.empty(),"reinitialization retained stale ready contexts");
 expect(!m.scene(p,adas::ViewId::Front,lights,{}).vision.model_ready,"invalid replacement manifest kept old model live");
 qputenv("ADAS_MODEL_MANIFEST",f.directory.filePath("manifest.json").toUtf8());invalid_vehicle_head=true;
 expect(!m.initialize(f.directory.path())&&contexts.size()==1,"one bad SDK head must reject only its context");
 bad=m.scene(p,adas::ViewId::Front,lights,{0,0,1,1},adas::SceneModelRole::Vehicle);
 expect(!bad.vision.model_ready&&bad.vision.observation.vehicle_status==adas::Status::Failed,"bad role initialization must stay Failed");
 good=m.scene(p,adas::ViewId::Front,lights,{0,0,1,1},adas::SceneModelRole::Traffic);
 expect(good.vision.model_ready&&good.vision.observation.crosswalk_status==adas::Status::Valid,"independent context failed to load after peer initialization error");
 invalid_vehicle_head=false;
 qunsetenv("ADAS_MODEL_MANIFEST");
}
}
extern "C" {
int rknn_init(rknn_context *c,void *model,uint32_t size,uint32_t,rknn_init_extend*){
 std::string bytes(static_cast<char*>(model),size);Context ctx;ctx.traffic=bytes.find("traffic.rknn")!=std::string::npos;
 const unsigned props=ctx.traffic?11:85;
 for(unsigned i=0;i<3;++i){const unsigned n=640/(8u<<i);ctx.heads[i].assign(3*props*n*n,ctx.traffic?-20.f:0.f);}
 // Same synthetic person in each model's own class order, actual raw vs sigmoid encoding.
 constexpr unsigned grid=80*80,cell=50*80+40;
 for(unsigned i=0;i<4;++i)ctx.heads[0][i*grid+cell]=ctx.traffic?0.f:.5f;
 ctx.heads[0][4*grid+cell]=ctx.traffic?3.f:.9f;
 ctx.heads[0][(ctx.traffic?6:5)*grid+cell]=ctx.traffic?3.f:.8f;
 *c=next_context++;contexts.emplace(*c,std::move(ctx));return 0;
}
int rknn_destroy(rknn_context c){expect(contexts.erase(c)==1,"wrong destroyed context");return 0;}
int rknn_query(rknn_context c,rknn_query_cmd q,void *info,uint32_t){
 auto &ctx=contexts.at(c);
 if(q==RKNN_QUERY_SDK_VERSION){std::strcpy(static_cast<rknn_sdk_version*>(info)->api_version,"2.3.2 (boundary)");return 0;}
 if(q==RKNN_QUERY_IN_OUT_NUM){*static_cast<rknn_input_output_num*>(info)={1,3};return 0;}
 auto &a=*static_cast<rknn_tensor_attr*>(info);unsigned i=a.index;a={};a.index=i;a.n_dims=4;a.dims[0]=1;
 a.type=RKNN_TENSOR_INT8;a.qnt_type=RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC;a.scale=1.f/255;
 if(q==RKNN_QUERY_INPUT_ATTR){a.fmt=RKNN_TENSOR_NHWC;a.dims[1]=a.dims[2]=640;a.dims[3]=3;a.n_elems=640*640*3;}
 else {a.fmt=RKNN_TENSOR_NCHW;a.dims[1]=3*(ctx.traffic?11:85)+(invalid_vehicle_head&&!ctx.traffic?1:0);a.dims[2]=a.dims[3]=640/(8u<<i);a.n_elems=ctx.heads[i].size();}
 a.size=a.n_elems;return 0;
}
int rknn_inputs_set(rknn_context c,uint32_t n,rknn_input inputs[]){
 expect(contexts.count(c)&&n==1&&inputs[0].size==640*640*3&&inputs[0].fmt==RKNN_TENSOR_NHWC&&inputs[0].type==RKNN_TENSOR_UINT8,"input contract wrong");
 expect(static_cast<unsigned char*>(inputs[0].buf)[0]==0,"role padding must be black");return 0;
}
int rknn_run(rknn_context c,rknn_run_extend*){
 bool t=contexts.at(c).traffic;++calls;role_runs.push_back(t);return (t?fail_traffic:fail_vehicle)?-1:0;
}
int rknn_outputs_get(rknn_context c,uint32_t n,rknn_output out[],rknn_output_extend*){
 expect(n==3,"output count wrong");auto &ctx=contexts.at(c);
 for(unsigned i=0;i<n;++i){out[i].buf=ctx.heads[i].data();out[i].size=ctx.heads[i].size()*sizeof(float);}return 0;
}
int rknn_outputs_release(rknn_context,uint32_t,rknn_output[]){++releases;return 0;}
}
namespace vision {
class RgaLibrary{};
RgaPreprocessor::RgaPreprocessor()=default;RgaPreprocessor::~RgaPreprocessor()=default;
bool RgaPreprocessor::initialize(){rga_available_=true;return true;}
bool RgaPreprocessor::rgb565_crop_to_rgb_strict(const unsigned char*,int,int,int,const cv::Rect&,cv::Mat &out,int w,int h,const cv::Rect &dst,unsigned char pad){
 last_pad=pad;out=cv::Mat(h,w,CV_8UC3,cv::Scalar(pad,pad,pad));out(dst).setTo(cv::Scalar(17,29,43));return true;
}
}
int main(int argc,char**argv){QCoreApplication app(argc,argv);try{test_roles();std::cout<<"dual runtime role isolation / 640 letterbox / output encodings / failure recovery: PASS\n";}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
