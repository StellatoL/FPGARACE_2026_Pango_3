#include "inference/neural_models.h"
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
  void open(const NeuralModelSpec &s,bool scene) {
    QFile file(s.path);if(!file.open(QIODevice::ReadOnly))throw std::runtime_error("model disappeared");
    auto bytes=file.readAll();checked(rknn_init(&context,bytes.data(),uint32_t(bytes.size()),0,nullptr),"rknn_init");
    rknn_sdk_version version{};checked(rknn_query(context,RKNN_QUERY_SDK_VERSION,&version,sizeof(version)),"SDK query");
    if(!QString::fromLatin1(version.api_version).startsWith(s.runtime_api_prefix))throw std::runtime_error("RKNN runtime does not match model manifest");
    rknn_input_output_num count{};checked(rknn_query(context,RKNN_QUERY_IN_OUT_NUM,&count,sizeof(count)),"tensor count");
    if(count.n_input!=1||count.n_output!=unsigned(scene?3:4))throw std::runtime_error("model tensor count mismatch");
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
      if(scene&&a.fmt!=RKNN_TENSOR_NCHW)throw std::runtime_error("scene output must be NCHW");
      tensors.push_back(std::move(tensor));
    }
    std::string error;if(!validate_neural_outputs(tensors,scene,s.classes.size(),error))throw std::runtime_error(error);
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
int light_priority(LightState s){return s==LightState::Red?3:s==LightState::Yellow?2:s==LightState::Green?1:0;}
bool inside(double x,double y,const Roi&r){return x>=r.x&&y>=r.y&&x<r.x+r.w&&y<r.y+r.h;}
}
struct NeuralModels::State {
  NeuralBundleSpec spec;Status scene_status=Status::Missing,lane_status=Status::Missing;
  QString scene_error="ADAS model bundle not configured",lane_error=scene_error;
  std::unique_ptr<Runtime> scene,lane;cv::Mat input;
#ifdef PANGO_HAVE_RGA
  vision::RgaPreprocessor rga;
#endif
  cv::Rect adapt(const FramePacket&p,ViewId view,const Roi &r,int w,int h,bool letterbox,cv::Rect &destination) {
    const auto v=view_rect(p.layout,view,p.width,p.height);
    if(!p||p.width<=0||p.height<=0||p.stride<p.width*2||size_t(p.stride)*(p.height-1)+size_t(p.width)*2>p.pixels->bytes.size()||v.w<=0||v.h<=0||!std::isfinite(r.x)||!std::isfinite(r.y)||!std::isfinite(r.w)||!std::isfinite(r.h)||r.x<0||r.y<0||r.w<=0||r.h<=0||r.x+r.w>1.000001||r.y+r.h>1.000001)throw std::runtime_error("invalid inference crop");
    const int x=int(r.x*v.w)&~1,y=int(r.y*v.h)&~1;
    const int cw=std::min(v.w-x,std::max(2,int(r.w*v.w)&~1)),ch=std::min(v.h-y,std::max(2,int(r.h*v.h)&~1));
    cv::Rect crop(v.x+x,v.y+y,cw,ch);destination={0,0,w,h};
    if(letterbox){const double scale=std::min(double(w)/cw,double(h)/ch);int dw=std::max(2,int(cw*scale)&~1),dh=std::max(2,int(ch*scale)&~1);destination={(w-dw)/2,(h-dh)/2,dw,dh};}
#ifdef PANGO_HAVE_RGA
    if(!rga.rgb565_crop_to_rgb_strict(p.pixels->bytes.data(),p.width,p.height,p.stride,crop,input,w,h,destination))throw std::runtime_error("RGA input adaptation failed; CPU fallback is disabled");
#else
    throw std::runtime_error("RGA unavailable in this build; ADAS CPU image fallback is disabled");
#endif
    return {x,y,cw,ch};
  }
};
NeuralModels::NeuralModels():state_(new State){}
NeuralModels::~NeuralModels()=default;
bool NeuralModels::initialize(const QString &root) {
  const QString override=qEnvironmentVariable("ADAS_MODEL_MANIFEST");
  const QString path=override.isEmpty()?QDir(root).filePath("models/adas/manifest.json"):override;
  QString error;
  if(!read_neural_manifest(path,state_->spec,error)) {
    state_->scene_error=state_->lane_error=error;
    state_->scene_status=state_->lane_status=QFileInfo::exists(path)?Status::Failed:Status::Missing;return false;
  }
  bool hardware=false;
#ifdef PANGO_HAVE_RGA
  hardware=state_->rga.initialize();
#endif
  if(!hardware){state_->scene_status=state_->lane_status=Status::Failed;state_->scene_error=state_->lane_error="RGA hardware self-check unavailable; CPU fallback disabled";return false;}
  auto load=[&](bool scene){
    try {auto model=std::make_unique<Runtime>();model->open(scene?state_->spec.scene:state_->spec.lane,scene);
      (scene?state_->scene:state_->lane)=std::move(model);
      (scene?state_->scene_status:state_->lane_status)=Status::Valid;
      (scene?state_->scene_error:state_->lane_error).clear();
    } catch(const std::exception&e){(scene?state_->scene_status:state_->lane_status)=Status::Failed;(scene?state_->scene_error:state_->lane_error)=QString::fromUtf8(e.what());}
  };
  load(true);load(false);return state_->scene&&state_->lane;
}
QString NeuralModels::description()const{return QStringLiteral("bundle=%1 scene=%2 lane=%3").arg(state_->spec.version).arg(int(state_->scene_status)).arg(int(state_->lane_status));}
NeuralSceneResult NeuralModels::scene(const FramePacket&p,ViewId view,const std::array<Roi,3>&lights,const Roi&nav,const Roi &roi) {
  NeuralSceneResult r;auto &v=r.vision;v.mode=DetectionMode::Adas;v.stamp=p.stamp;v.view=view;
  const auto region=view_rect(p.layout,view,p.width,p.height);
  auto &o=v.observation;o.stamp=p.stamp;o.view=view;o.width=region.w;o.height=region.h;o.components=ObjectObservation::All;o.signal_observation=false;
  o.traffic_status=o.vehicle_status=o.roadblock_status=o.signal_status=state_->scene_status;
  r.navigation.stamp=p.stamp;r.navigation.components=GeometryObservation::Navigation;
  r.navigation.status=state_->scene_status;r.navigation.width=region.w;r.navigation.height=region.h;
  const double start=monotonic_seconds();
  try {
    if(!state_->scene)throw std::runtime_error(state_->scene_error.toStdString());
    cv::Rect dst;const auto crop=state_->adapt(p,view,roi,640,640,true,dst);auto outputs=state_->scene->run(state_->input);
    std::vector<NeuralDetection> found;std::string decode_error;
    if(!decode_neural_scene(outputs,state_->spec.scene,found,decode_error))throw std::runtime_error(decode_error);
    o.has_lane_lights=true;o.lane_lights.fill(LightState::Unknown);
    for(const auto &d:found) {
      if(d.class_id<0||d.class_id>=int(state_->spec.scene.classes.size()))continue;
      const auto &name=state_->spec.scene.classes[d.class_id];
      Box box{float(crop.x+(d.box.left-dst.x)*double(crop.width)/dst.width),float(crop.y+(d.box.top-dst.y)*double(crop.height)/dst.height),
        float(crop.x+(d.box.right-dst.x)*double(crop.width)/dst.width),float(crop.y+(d.box.bottom-dst.y)*double(crop.height)/dst.height)};
      box.left=std::clamp(box.left,0.f,float(region.w));box.right=std::clamp(box.right,0.f,float(region.w));
      box.top=std::clamp(box.top,0.f,float(region.h));box.bottom=std::clamp(box.bottom,0.f,float(region.h));
      if(box.right-box.left<=1||box.bottom-box.top<=1)continue;
      const double cx=(box.left+box.right)/(2*region.w),cy=(box.top+box.bottom)/(2*region.h);
      int kind=name=="person"?1:name=="vehicle"?10:(name=="cone"||name=="barrier"||name=="debris")?11:-1;
      if(kind>=0)o.obstacles.push_back({kind,-1,box,d.confidence,-1});
      if(name=="crosswalk")o.has_zebra=true;
      auto lamp=[&](const std::string&base){return name==base||name==base+"_left"||name==base+"_straight"||name==base+"_right";};
      const LightState light=lamp("light_red")?LightState::Red:lamp("light_yellow")?LightState::Yellow:lamp("light_green")?LightState::Green:LightState::Unknown;
      if(light!=LightState::Unknown) {
        if(light_priority(light)>light_priority(o.light))o.light=light;
        int explicit_lane=name.find("_left")!=std::string::npos?0:name.find("_straight")!=std::string::npos?1:name.find("_right")!=std::string::npos?2:-1;
        for(int i=0;i<3;++i)if((explicit_lane<0||explicit_lane==i)&&inside(cx,cy,lights[i])&&light_priority(light)>light_priority(o.lane_lights[i]))o.lane_lights[i]=light;
      }
      if(view==ViewId::Front&&inside(cx,cy,nav)&&(name=="nav_left"||name=="nav_straight"||name=="nav_right")&&d.confidence>=.5f) {
        auto &n=r.navigation.nav;
        if(d.confidence>n.confidence)n={true,name=="nav_left"?Direction::Left:name=="nav_right"?Direction::Right:Direction::Straight,d.confidence,box};
      }
      OverlayBoxResult overlay;overlay.left=int(box.left);overlay.top=int(box.top);overlay.right=int(box.right);overlay.bottom=int(box.bottom);
      overlay.class_id=kind;overlay.confidence=d.confidence;overlay.label=name;overlay.color_r=kind==1||kind==11?255:0;overlay.color_b=kind==10?255:0;
      v.overlays.push_back(std::move(overlay));
    }
    for(const auto &person:o.obstacles)if(person.class_id==1)for(size_t i=0;i<v.overlays.size();++i) {
      const auto &z=v.overlays[i];if(z.label!="crosswalk")continue;
      if(person.box.right>z.left&&person.box.left<z.right&&person.box.bottom>=z.top&&person.box.bottom<=z.bottom)o.person_on_zebra=true;
    }
    v.model_ready=true;
  } catch(const std::exception&e) {
    v.error_message=e.what();o.error=e.what();
    if(state_->scene)o.traffic_status=o.vehicle_status=o.roadblock_status=o.signal_status=Status::Failed;
    r.navigation.status=o.traffic_status;v.overlays.clear();o.obstacles.clear();
  }
  o.infer_ms=float((monotonic_seconds()-start)*1000);return r;
}
GeometryObservation NeuralModels::lane(const FramePacket&p) {
  GeometryObservation o;o.stamp=p.stamp;o.components=GeometryObservation::Lane;
  const auto view=view_rect(p.layout,ViewId::Front,p.width,p.height);o.width=view.w;o.height=view.h;o.status=state_->lane_status;
  o.speed.reason="visual speed deferred; simulation/external speed only";const double start=monotonic_seconds();
  try {
    if(!state_->lane)throw std::runtime_error(state_->lane_error.toStdString());
    cv::Rect dst;const auto crop=state_->adapt(p,ViewId::Front,state_->spec.road_roi,512,256,false,dst);
    const Roi actual{double(crop.x)/view.w,double(crop.y)/view.h,double(crop.width)/view.w,double(crop.height)/view.h};
    auto data=state_->lane->run(state_->input);LaneTensorData lane{data[0],data[1],data[2],data[3]};std::string error;
    if(!decode_neural_lane(lane,actual,view.w,view.h,o.lane,error))throw std::runtime_error(error);
    o.status=Status::Valid;
  } catch(const std::exception&e) {if(state_->lane)o.status=Status::Failed;o.speed.reason=e.what();}
  o.process_ms=float((monotonic_seconds()-start)*1000);return o;
}
}
