#include "inference/neural_models.h"
#include "inference/neural_scene_adapter.h"
#include <QFile>
#include <QDir>
#include <QFileInfo>
#include <rknn_api.h>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#ifdef PANGO_HAVE_RGA
#include "inference/rga_preprocessor.h"
#endif
namespace adas {
namespace {
void checked(int code,const char *action) {if(code<0)throw std::runtime_error(std::string(action)+": "+std::to_string(code));}
struct Runtime {
  rknn_context context=0;std::vector<rknn_tensor_attr> outputs;
  ~Runtime(){if(context)rknn_destroy(context);}
  void open(const NeuralModelSpec &s) {
    QFile file(s.path);if(!file.open(QIODevice::ReadOnly))throw std::runtime_error("model disappeared");
    auto bytes=file.readAll();checked(rknn_init(&context,bytes.data(),uint32_t(bytes.size()),0,nullptr),"rknn_init");
    rknn_sdk_version version{};checked(rknn_query(context,RKNN_QUERY_SDK_VERSION,&version,sizeof(version)),"SDK query");
    if(!QString::fromLatin1(version.api_version).startsWith(s.runtime_api_prefix))throw std::runtime_error("RKNN runtime does not match model manifest");
    rknn_input_output_num count{};checked(rknn_query(context,RKNN_QUERY_IN_OUT_NUM,&count,sizeof(count)),"tensor count");
    if(count.n_input!=1||count.n_output!=3)throw std::runtime_error("model tensor count mismatch");
    rknn_tensor_attr input{};checked(rknn_query(context,RKNN_QUERY_INPUT_ATTR,&input,sizeof(input)),"input query");
    const bool shape=input.n_dims==4&&input.dims[0]==1&&
      ((input.fmt==RKNN_TENSOR_NHWC&&input.dims[1]==unsigned(s.height)&&input.dims[2]==unsigned(s.width)&&input.dims[3]==3)||
       (input.fmt==RKNN_TENSOR_NCHW&&input.dims[1]==3&&input.dims[2]==unsigned(s.height)&&input.dims[3]==unsigned(s.width)));
    if(!shape)throw std::runtime_error("model input shape mismatch");
    outputs.resize(count.n_output);bool quantized=input.qnt_type!=RKNN_TENSOR_QNT_NONE;
    std::vector<NeuralTensorSpec> tensors;
    for(unsigned i=0;i<count.n_output;++i) {
      auto &a=outputs[i];a.index=i;checked(rknn_query(context,RKNN_QUERY_OUTPUT_ATTR,&a,sizeof(a)),"output query");
      quantized=quantized||a.qnt_type!=RKNN_TENSOR_QNT_NONE;
      NeuralTensorSpec tensor;tensor.name=QString::fromLatin1(a.name);
      for(unsigned d=0;d<a.n_dims;++d)tensor.shape.push_back(a.dims[d]);
      if(a.fmt!=RKNN_TENSOR_NCHW)throw std::runtime_error("scene output must be NCHW");
      tensors.push_back(std::move(tensor));
    }
    std::string error;if(!validate_neural_outputs(tensors,s.classes.size(),s.width,s.height,error))throw std::runtime_error(error);
    if(!quantized)throw std::runtime_error("manifest INT8 declaration disagrees with model tensors");
  }
  std::vector<std::vector<float>> run(cv::Mat &rgb) {
    rknn_input input{};input.index=0;input.buf=rgb.data;input.size=uint32_t(rgb.total()*3);
    input.type=RKNN_TENSOR_UINT8;input.fmt=RKNN_TENSOR_NHWC;
    checked(rknn_inputs_set(context,1,&input),"set input");checked(rknn_run(context,nullptr),"run");
    std::vector<rknn_output> out(outputs.size());
    for(size_t i=0;i<out.size();++i){out[i].index=unsigned(i);out[i].want_float=1;}
    checked(rknn_outputs_get(context,unsigned(out.size()),out.data(),nullptr),"get outputs");
    struct Release {rknn_context c;std::vector<rknn_output>&o;~Release(){rknn_outputs_release(c,unsigned(o.size()),o.data());}} release{context,out};
    std::vector<std::vector<float>> result;
    for(size_t i=0;i<out.size();++i) {
      if(!out[i].buf||out[i].size<outputs[i].n_elems*sizeof(float))throw std::runtime_error("short RKNN output");
      const float *p=static_cast<float*>(out[i].buf);
      result.emplace_back(p,p+outputs[i].n_elems);
      if(!std::all_of(result.back().begin(),result.back().end(),[](float v){return std::isfinite(v);}))throw std::runtime_error("nonfinite RKNN output");
    }
    return result;
  }
};
}
struct NeuralModels::State {
  NeuralBundleSpec spec;
  Status traffic_status=Status::Missing,vehicle_status=Status::Missing;
  QString traffic_error="ADAS model bundle not configured",vehicle_error=traffic_error;
  std::unique_ptr<Runtime> traffic,vehicle;cv::Mat input;
#ifdef PANGO_HAVE_RGA
  vision::RgaPreprocessor rga;
#endif
  cv::Rect adapt(const FramePacket&p,ViewId view,const Roi &r,int w,int h,cv::Rect &destination) {
    const auto v=view_rect(p.layout,view,p.width,p.height);
    if(!p||p.width<=0||p.height<=0||p.stride<p.width*2||size_t(p.stride)*(p.height-1)+size_t(p.width)*2>p.pixels->bytes.size()||v.w<=0||v.h<=0||!std::isfinite(r.x)||!std::isfinite(r.y)||!std::isfinite(r.w)||!std::isfinite(r.h)||r.x<0||r.y<0||r.w<=0||r.h<=0||r.x+r.w>1.000001||r.y+r.h>1.000001)throw std::runtime_error("invalid inference crop");
    const int x=int(r.x*v.w)&~1,y=int(r.y*v.h)&~1;
    const int cw=std::min(v.w-x,std::max(2,int(r.w*v.w)&~1)),ch=std::min(v.h-y,std::max(2,int(r.h*v.h)&~1));
    cv::Rect crop(v.x+x,v.y+y,cw,ch);destination={0,0,w,h};
    {const double scale=std::min(double(w)/cw,double(h)/ch);int dw=std::max(2,int(cw*scale)&~1),dh=std::max(2,int(ch*scale)&~1);destination={(w-dw)/2,(h-dh)/2,dw,dh};}
#ifdef PANGO_HAVE_RGA
    if(!rga.rgb565_crop_to_rgb_strict(p.pixels->bytes.data(),p.width,p.height,p.stride,crop,input,w,h,destination,0))throw std::runtime_error("RGA input adaptation failed; CPU fallback is disabled");
#else
    throw std::runtime_error("RGA unavailable in this build; ADAS CPU image fallback is disabled");
#endif
    return {x,y,cw,ch};
  }
};
NeuralModels::NeuralModels():state_(new State){}
NeuralModels::~NeuralModels()=default;
bool NeuralModels::initialize(const QString &root) {
  // A failed replacement must never leave the previously loaded weights live.
  state_=std::make_unique<State>();
  const QString override=qEnvironmentVariable("ADAS_MODEL_MANIFEST");
  const QString path=override.isEmpty()?QDir(root).filePath("models/adas/manifest.json"):override;
  QString error;
  if(!read_neural_manifest(path,state_->spec,error)) {
    state_->traffic_error=state_->vehicle_error=error;
    state_->traffic_status=state_->vehicle_status=
      QFileInfo::exists(path)?Status::Failed:Status::Missing;
    return false;
  }
  bool hardware=false;
#ifdef PANGO_HAVE_RGA
  hardware=state_->rga.initialize();
#endif
  if(!hardware){
    const QString reason="RGA hardware self-check unavailable; CPU fallback disabled";
    state_->traffic_status=state_->vehicle_status=Status::Failed;
    state_->traffic_error=state_->vehicle_error=reason;
    return false;
  }
  auto load=[](const NeuralModelSpec &spec,std::unique_ptr<Runtime> &runtime,Status &status,QString &reason){
    try {auto model=std::make_unique<Runtime>();model->open(spec);runtime=std::move(model);status=Status::Valid;reason.clear();}
    catch(const std::exception&e){status=Status::Failed;reason=QString::fromUtf8(e.what());}
  };
  load(state_->spec.vehicle,state_->vehicle,state_->vehicle_status,state_->vehicle_error);
  load(state_->spec.traffic,state_->traffic,state_->traffic_status,state_->traffic_error);
  return state_->traffic&&state_->vehicle;
}
QString NeuralModels::description()const{
  return QStringLiteral("bundle=%1 vehicle=%2 traffic=%3 lane=classical input=640x640")
    .arg(state_->spec.version).arg(int(state_->vehicle_status)).arg(int(state_->traffic_status));
}
NeuralSceneResult NeuralModels::scene(const FramePacket&p,ViewId view,const std::array<Roi,3>&lights,const Roi &roi,SceneModelRole role) {
  NeuralSceneResult r;r.role=role;auto &v=r.vision;v.mode=DetectionMode::Adas;v.stamp=p.stamp;v.view=view;
  const auto region=view_rect(p.layout,view,p.width,p.height);
  auto &o=v.observation;o.stamp=p.stamp;o.view=view;o.width=region.w;o.height=region.h;o.signal_observation=false;
  const bool traffic=role==SceneModelRole::Traffic,vehicle=role==SceneModelRole::Vehicle;
  if(!traffic&&!vehicle){o.components=0;o.error=v.error_message="invalid model role";return r;}
  Runtime *runtime=vehicle?state_->vehicle.get():state_->traffic.get();
  const auto &spec=vehicle?state_->spec.vehicle:state_->spec.traffic;
  const Status status=vehicle?state_->vehicle_status:state_->traffic_status;
  const auto &reason=vehicle?state_->vehicle_error:state_->traffic_error;
  o.components=vehicle?ObjectObservation::Combined:
    ObjectObservation::Crosswalk|(view==ViewId::Front?ObjectObservation::Signal:0);
  const auto statuses=[&](Status value){
    if(vehicle)o.traffic_status=o.vehicle_status=value;
    else{o.crosswalk_status=value;if(view==ViewId::Front)o.signal_status=value;}
  };
  statuses(status);
  const double start=monotonic_seconds();
  try {
    if(!runtime)throw std::runtime_error(reason.toStdString());
    cv::Rect dst;const auto crop=state_->adapt(p,view,roi,spec.width,spec.height,dst);
    r.input_mapping={true,spec.width,spec.height,
      {region.x+crop.x,region.y+crop.y,crop.width,crop.height},{dst.x,dst.y,dst.width,dst.height}};
    auto outputs=runtime->run(state_->input);
    std::vector<NeuralDetection> found;std::string decode_error;
    if(!decode_neural_scene(outputs,spec,found,decode_error,&r.decode))throw std::runtime_error(decode_error);
    adapt_neural_detections(found,spec,role,{crop.x,crop.y,crop.width,crop.height},
      {dst.x,dst.y,dst.width,dst.height},lights,v);
    v.model_ready=true;
  } catch(const std::exception&e) {
    v.error_message=e.what();o.error=e.what();
    if(runtime)statuses(Status::Failed);
    v.overlays.clear();o.obstacles.clear();
  }
  o.infer_ms=float((monotonic_seconds()-start)*1000);return r;
}
}
