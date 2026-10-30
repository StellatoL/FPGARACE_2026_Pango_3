// Offline real-media acceptance utility. Production input remains FPGA only.
#include "common/frame_packet.h"
#include "core/adas_engine.h"
#include "core/ground_calibration.h"
#include "core/manual_controls.h"
#include "core/road_geometry.h"
#include "core/tracked_overlays.h"
#include "core/view_tracker.h"
#include "inference/neural_models.h"
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {
using namespace adas;
using Clock = std::chrono::steady_clock;
const std::vector<std::string> kClasses{"person","vehicle","crosswalk","light_red","light_green","light_black","light_yellow"};
QString qs(const std::string &s) { return QString::fromStdString(s); }
QJsonValue num(double v) { return std::isfinite(v) ? QJsonValue(v) : QJsonValue(); }
double elapsed(Clock::time_point start) { return std::chrono::duration<double,std::milli>(Clock::now()-start).count(); }
std::string status(Status v) { return v==Status::Valid?"valid":v==Status::Failed?"failed":"missing"; }
std::string light(LightState v) { return v==LightState::Red?"red":v==LightState::Green?"green":v==LightState::Yellow?"yellow":v==LightState::Black?"black":"unknown"; }
std::string line(LineType v) { return v==LineType::Solid?"solid":v==LineType::Dashed?"dashed":"unknown"; }
std::string direction(Direction v) { return v==Direction::Left?"left":v==Direction::Right?"right":v==Direction::Straight?"straight":"none"; }
std::string turn(TurnSignal v) { return v==TurnSignal::Left?"left":v==TurnSignal::Right?"right":"none"; }
std::string kind(int v) { return v==1?"person":v==10?"vehicle":v==11?"roadblock":"other"; }
std::string roleName(SceneModelRole role) {
  return role==SceneModelRole::Vehicle?"vehicle":"traffic";
}
QJsonArray boxJson(const Box &b) { return {num(b.left),num(b.top),num(b.right),num(b.bottom)}; }
QJsonObject stampJson(const Stamp &s) {
  return {{"generation",num(s.generation)},{"sequence",num(s.sequence)},
    {"source_s",num(s.source_s)},{"receive_s",num(s.receive_s)},{"static_image",s.static_image}};
}
QString hashFile(const QString &path) {
  QFile f(path); if(!f.open(QIODevice::ReadOnly)) return {};
  QCryptographicHash h(QCryptographicHash::Sha256); return h.addData(&f)?QString::fromLatin1(h.result().toHex()):QString();
}
QJsonObject fileRecord(const QString &path) {
  QFileInfo f(path);return {{"path",f.absoluteFilePath()},{"present",f.isFile()},
    {"bytes",num(f.size())},{"sha256",hashFile(path)},{"modified_utc",f.lastModified().toUTC().toString(Qt::ISODateWithMs)}};
}
QJsonObject modelRecord(const NeuralModelSpec &spec,SceneModelRole role) {
  QJsonArray classes,selected,anchors,shapes;
  for(const auto &name:spec.classes)classes.append(qs(name));
  for(const int id:spec.selected_classes)selected.append(id);
  for(const auto &head:spec.anchors){QJsonArray values;for(const float value:head)values.append(num(value));anchors.append(values);}
  for(const int stride:{8,16,32})shapes.append(QJsonArray{1,int(3*(spec.classes.size()+5)),spec.height/stride,spec.width/stride});
  return {{"role",qs(roleName(role))},{"model",fileRecord(spec.path)},{"declared_sha256",spec.sha256},
    {"runtime_api_prefix",spec.runtime_api_prefix},{"output_contract",spec.output},{"classes",classes},
    {"selected_classes",selected},{"empty_selection_means_all",true},{"anchors",anchors},
    {"input",QJsonObject{{"width",spec.width},{"height",spec.height},{"layout","NHWC"},{"format","RGB8"},
      {"mean",QJsonArray{0,0,0}},{"std",QJsonArray{255,255,255}}}},{"expected_head_shapes",shapes}};
}
void jsonLine(std::ofstream &stream,const QJsonObject &obj) {
  const auto bytes=QJsonDocument(obj).toJson(QJsonDocument::Compact);
  stream.write(bytes.constData(),bytes.size());stream.put('\n');
  if(!stream)throw std::runtime_error("cannot write diagnostic JSONL");
}
void writeJson(const std::filesystem::path &path,const QJsonObject &obj) {
  std::ofstream stream(path);const auto bytes=QJsonDocument(obj).toJson(QJsonDocument::Indented);
  stream.write(bytes.constData(),bytes.size());if(!stream)throw std::runtime_error("cannot write summary");
}
std::vector<std::string> split(const std::string &s) {
  std::vector<std::string> out;std::istringstream in(s);std::string item;
  while(std::getline(in,item,',')){if(item.empty())throw std::runtime_error("empty list entry");out.push_back(item);}return out;
}
double readNumber(const std::string &s) {
  size_t used=0;double v=std::stod(s,&used);
  if(used!=s.size()||!std::isfinite(v)||v<0)throw std::runtime_error("invalid nonnegative number: "+s);
  return v;
}
unsigned readUnsigned(const std::string &s) {
  double v=readNumber(s);if(v>10000000||std::floor(v)!=v)throw std::runtime_error("invalid bounded integer: "+s);return unsigned(v);
}
ManualAction action(const std::string &name) {
  static const std::map<std::string,ManualAction> actions{
    {"go",ManualAction::Drive},{"stop",ManualAction::Stop},{"force-drive",ManualAction::ForceDrive},
    {"nav-left",ManualAction::NavigateLeft},{"nav-straight",ManualAction::NavigateStraight},{"nav-right",ManualAction::NavigateRight},
    {"force-left",ManualAction::ForceChangeLeft},{"force-right",ManualAction::ForceChangeRight},
    {"clear-nav",ManualAction::ClearNavigation},{"cycle-nav",ManualAction::CycleNavigation},
    {"signal-left",ManualAction::SignalLeft},{"signal-right",ManualAction::SignalRight},
    {"toggle-force-left",ManualAction::ToggleForceLeft},{"toggle-force-right",ManualAction::ToggleForceRight},{"toggle-stop",ManualAction::ToggleStop}};
  const auto found=actions.find(name);if(found==actions.end())throw std::runtime_error("unknown manual command: "+name);return found->second;
}
struct Command { double at=0;std::string name;bool executed=false; };
struct Options {
  std::string input,output,root=".",object_dir,calibration;double sample_fps=10,speed_kmh=0;
  Roi signal_roi{0,0,1,.5};
  unsigned max_frames=120,overlay_every=10,drop_every=0;bool intersection=false,overwrite=false,continue_on_error=false;
  std::vector<Command> commands;std::vector<std::string> expect;
};
Options parse(int argc,char **argv) {
  Options o;
  for(int i=1;i<argc;++i){
    const std::string key=argv[i];
    auto value=[&](){if(i+1>=argc)throw std::runtime_error("missing value for "+key);return std::string(argv[++i]);};
    if(key=="--input")o.input=value();else if(key=="--output")o.output=value();else if(key=="--root")o.root=value();
    else if(key=="--object-dir")o.object_dir=value();else if(key=="--calibration")o.calibration=value();
    else if(key=="--sample-fps")o.sample_fps=readNumber(value());else if(key=="--speed-kmh")o.speed_kmh=readNumber(value());
    else if(key=="--max-frames")o.max_frames=readUnsigned(value());else if(key=="--overlay-every")o.overlay_every=readUnsigned(value());
    else if(key=="--drop-person-every")o.drop_every=readUnsigned(value());else if(key=="--expect")o.expect=split(value());
    else if(key=="--signal-roi"){
      const auto values=split(value());if(values.size()!=4)throw std::runtime_error("--signal-roi requires x,y,w,h");
      o.signal_roi={readNumber(values[0]),readNumber(values[1]),readNumber(values[2]),readNumber(values[3])};
      const auto &r=o.signal_roi;if(r.w<=0||r.h<=0||r.x+r.w>1||r.y+r.h>1)throw std::runtime_error("signal ROI must lie inside normalized 1280x720 canvas");
    }else if(key=="--intersection")o.intersection=true;else if(key=="--overwrite")o.overwrite=true;
    else if(key=="--continue-on-inference-error")o.continue_on_error=true;
    else if(key=="--commands")for(auto item:split(value())){
      Command c;const auto colon=item.find(':');if(colon!=std::string::npos){c.at=readNumber(item.substr(0,colon));item=item.substr(colon+1);}
      c.name=item;(void)action(c.name);o.commands.push_back(c);
    }else if(key=="--help"||key=="-h"){
      std::cout<<"See tools/run_scene_diagnostics.sh --help; --input and --output are required.\n";std::exit(0);
    }else throw std::runtime_error("unknown diagnostic option: "+key);
  }
  if(o.input.empty()||o.output.empty())throw std::runtime_error("--input and --output are required");
  if(o.sample_fps<=0||o.sample_fps>1000||o.speed_kmh>300)throw std::runtime_error("sample fps or explicit speed outside supported bounds");
  const std::set<std::string> extra{"roadblock","left-solid","left-dashed","right-solid","right-dashed","lane","signal-red","signal-green"};
  for(const auto &e:o.expect)if(std::find(kClasses.begin(),kClasses.end(),e)==kClasses.end()&&!extra.count(e))throw std::runtime_error("unknown scene expectation: "+e);
  std::stable_sort(o.commands.begin(),o.commands.end(),[](const auto&a,const auto&b){return a.at<b.at;});return o;
}
QJsonObject egoJson(const EgoState &e) {
  return {{"layout","single"},{"lane",int(e.lane)},{"automatic",e.automatic},{"manual_stop",e.manual_stop},
    {"force_drive",e.force_drive},{"force_lane_change",qs(turn(e.force_lane_change))},{"signal",qs(turn(e.signal))},
    {"manual_nav_enabled",e.manual_nav_enabled},{"manual_nav",qs(direction(e.manual_nav))},
    {"lane_change_request_s",num(e.lane_change_request_s)},{"intersection_mode",e.intersection_mode},
    {"speed_source","explicit_simulation"},{"manual_speed_kmh",num(e.manual_speed_kmh)}};
}
QJsonObject decisionJson(const AdasDecision &d) {
  QJsonArray points;for(unsigned i=0;i<d.local_plan.point_count&&i<d.local_plan.points.size();++i)
    points.append(QJsonArray{num(d.local_plan.points[i].x),num(d.local_plan.points[i].y)});
  return {{"stop_required",d.stop_required},{"brake_now",d.brake_now},{"auto_driving",d.auto_driving},
    {"slowdown_warning",d.slowdown_warning},
    {"alarm_red_run",d.alarm_red_run},{"alarm_illegal_lane_change",d.alarm_illegal_lane_change},
    {"alarm_overspeed",d.alarm_overspeed},{"alarm_collision",d.alarm_collision},{"alarm_lane_departure",d.alarm_lane_departure},
    {"lane_change_allowed",d.lane_change_allowed},{"speed_valid",d.speed_valid},{"speed_kmh",num(d.speed_kmh)},
    {"speed_source",int(d.speed_source)},{"nearest_obstacle_m",num(d.nearest_obstacle_m)},{"ttc_s",num(d.ttc_s)},
    {"arrow",qs(direction(d.arrow))},{"light",qs(light(d.light))},{"led_mask",int(d.led_mask)},
    {"status_text",qs(d.status_text)},{"hint_text",qs(d.hint_text)},
    {"local_plan",QJsonObject{{"valid",d.local_plan.valid},{"maneuver",int(d.local_plan.maneuver)},
      {"reason",qs(d.local_plan.reason)},{"points",points}}}};
}
QJsonObject laneJson(const GeometryObservation &g) {
  const auto &l=g.lane;return {{"status",qs(status(g.status))},{"components",int(g.components)},
    {"left_valid",l.left_valid},{"right_valid",l.right_valid},{"left_type",qs(line(l.left_type))},{"right_type",qs(line(l.right_type))},
    {"left_confidence",num(l.left_conf)},{"right_confidence",num(l.right_conf)},
    {"left_bottom_x",num(l.left_x)},{"right_bottom_x",num(l.right_x)},
    {"left_top_x",num(l.left_top_x)},{"right_top_x",num(l.right_top_x)},
    {"top_y",num(l.top_y)},{"bottom_y",num(l.bottom_y)},{"center_offset",num(l.center_offset)},
    {"navigation_produced",g.nav.detected},{"visual_speed_produced",g.speed.valid},{"process_ms",num(g.process_ms)}};
}
using Counts=std::map<std::string,unsigned>;
Counts modelCounts(const VisionFrameResult &v) {
  Counts c;for(const auto &name:kClasses)c[name]=0;c["roadblock"]=0;
  for(const auto &b:v.overlays){++c[b.label];if(b.class_id==10&&b.label!="vehicle")++c["vehicle"];if(b.class_id==11)++c["roadblock"];}return c;
}
unsigned count(const Counts &c,const std::string &key) { const auto i=c.find(key);return i==c.end()?0:i->second; }
QJsonObject countsJson(const Counts &c) { QJsonObject j;for(const auto &p:c)j[qs(p.first)]=int(p.second);return j; }
void addCounts(Counts &to,const Counts &from) { for(const auto &p:from)to[p.first]+=p.second; }
QJsonArray rawJson(const VisionFrameResult &v) {
  QJsonArray a;for(const auto &b:v.overlays)a.append(QJsonObject{{"model_class",qs(b.label)},
    {"adapter_class_id",b.class_id},{"confidence",num(b.confidence)},
    {"box_canvas_xyxy",QJsonArray{b.left,b.top,b.right,b.bottom}}});return a;
}
QJsonObject statuses(const ObjectObservation &o) {
  return {{"traffic",qs(status(o.traffic_status))},{"vehicle",qs(status(o.vehicle_status))},
    {"roadblock",qs(status(o.roadblock_status))},{"crosswalk",qs(status(o.crosswalk_status))},
    {"signal",qs(status(o.signal_status))},{"error",qs(o.error)}};
}
QJsonObject inferenceJson(const NeuralSceneResult &r) {
  const auto &m=r.input_mapping;
  const auto rect=[](const ViewRect &v){return QJsonArray{v.x,v.y,v.w,v.h};};
  return {{"role",qs(roleName(r.role))},{"stamp",stampJson(r.vision.observation.stamp)},
    {"model_ready",r.vision.model_ready},{"error",qs(r.vision.error_message)},
    {"components",int(r.vision.observation.components)},{"statuses",statuses(r.vision.observation)},
    {"infer_ms",num(r.vision.observation.infer_ms)},
    {"input_prepared",m.prepared},{"input_size",QJsonArray{m.width,m.height}},
    {"source_crop_canvas_xywh",rect(m.source_crop)},{"letterbox_content_xywh",rect(m.content)},
    {"candidate_count",num(r.decode.candidate_count)},{"nms_kept",num(r.decode.nms_kept)},
    {"nms_comparisons",num(r.decode.nms_comparisons)},{"probability_roundoff_clamps",num(r.decode.probability_clamps)}};
}
struct RoleStats {
  unsigned samples=0,ready=0,failed=0;
  size_t candidates=0,kept=0,comparisons=0,clamps=0;
  double infer_ms=0,max_infer_ms=0;
  Counts errors;
  void add(const NeuralSceneResult &result) {
    ++samples;if(result.vision.model_ready)++ready;else{++failed;++errors[result.vision.error_message];}
    candidates+=result.decode.candidate_count;kept+=result.decode.nms_kept;
    comparisons+=result.decode.nms_comparisons;clamps+=result.decode.probability_clamps;
    infer_ms+=result.vision.observation.infer_ms;max_infer_ms=std::max(max_infer_ms,double(result.vision.observation.infer_ms));
  }
  QJsonObject json()const {
    return {{"samples",int(samples)},{"ready_samples",int(ready)},{"failed_samples",int(failed)},
      {"candidate_count",num(candidates)},{"nms_kept",num(kept)},{"nms_comparisons",num(comparisons)},
      {"probability_roundoff_clamps",num(clamps)},{"infer_ms_total",num(infer_ms)},
      {"infer_ms_mean",num(samples?infer_ms/samples:0)},{"infer_ms_max",num(max_infer_ms)},{"errors",countsJson(errors)}};
  }
};
QJsonArray trackedJson(const ObjectObservation &o) {
  QJsonArray a;for(const auto &b:o.obstacles)a.append(QJsonObject{{"class",qs(kind(b.class_id))},{"class_id",b.class_id},
    {"track_id",QString::number(b.track_id)},{"predicted",b.predicted},{"track_hits",int(b.track_hits)},
    {"last_observed_s",num(b.last_observed_s)},{"age_s",num(o.stamp.receive_s-b.last_observed_s)},
    {"confidence",num(b.confidence)},{"box_canvas_xyxy",boxJson(b.box)},{"distance_m",num(b.distance_m)}});return a;
}
cv::Mat canvas(const cv::Mat &source,QJsonObject &mapping) {
  if(source.empty()||source.type()!=CV_8UC3)throw std::runtime_error("expected decoded BGR image");
  const double scale=std::min(1280./source.cols,720./source.rows);
  const int w=std::clamp(int(std::lround(source.cols*scale)),1,1280),h=std::clamp(int(std::lround(source.rows*scale)),1,720);
  const int x=(1280-w)/2,y=(720-h)/2;cv::Mat result(720,1280,CV_8UC3,cv::Scalar(0,0,0));
  cv::resize(source,result(cv::Rect(x,y,w,h)),cv::Size(w,h),0,0,scale<1?cv::INTER_AREA:cv::INTER_LINEAR);
  mapping={{"source_width",source.cols},{"source_height",source.rows},{"canvas_width",1280},{"canvas_height",720},
    {"scaled_width",w},{"scaled_height",h},{"pad_x",x},{"pad_y",y},{"scale_x",num(double(w)/source.cols)},
    {"scale_y",num(double(h)/source.rows)},{"aspect_preserved",true},{"rgb565_roundtrip",true}};return result;
}
void drawOverlay(cv::Mat &image,const VisionFrameResult &v,const VisionFrameResult &signal,
                 const GeometryObservation &g,const AdasDecision &decision,unsigned seq,double time,unsigned removed) {
  for(const auto &b:v.overlays){
    const cv::Scalar color=b.predicted?cv::Scalar(40,190,255):cv::Scalar(b.color_b,b.color_g,b.color_r);
    cv::rectangle(image,cv::Point(b.left,b.top),cv::Point(b.right,b.bottom),color,b.predicted?1:2);
    std::ostringstream label;label<<b.label<<' '<<std::fixed<<std::setprecision(2)<<b.confidence;
    if(b.track_id>=0)label<<" id="<<b.track_id<<" hits="<<b.track_hits<<" age="<<std::setprecision(2)<<time-b.last_observed_s<<(b.predicted?" predicted":" measured");
    cv::putText(image,label.str(),{std::max(0,b.left),std::max(18,b.top-5)},cv::FONT_HERSHEY_SIMPLEX,.4,color,1);
  }
  for(const auto &b:signal.overlays)if(b.label.find("light_")==0||b.class_id==12||b.label=="crosswalk"){
    cv::rectangle(image,{b.left,b.top},{b.right,b.bottom},cv::Scalar(255,60,255),2);
    cv::putText(image,"traffic:"+b.label,{std::max(0,b.left),std::max(18,b.bottom+15)},cv::FONT_HERSHEY_SIMPLEX,.4,{255,60,255},1);
  }
  const auto &l=g.lane;
  if(l.left_valid)cv::line(image,{int(l.left_top_x),int(l.top_y)},{int(l.left_x),int(l.bottom_y)},{0,255,255},3);
  if(l.right_valid)cv::line(image,{int(l.right_top_x),int(l.top_y)},{int(l.right_x),int(l.bottom_y)},{0,255,255},3);
  std::ostringstream label;label<<"seq="<<seq<<" media="<<std::fixed<<std::setprecision(2)<<time
    <<" lanes="<<line(l.left_type)<<'/'<<line(l.right_type)<<" light="<<light(decision.light)
    <<" stop="<<decision.stop_required<<" drive="<<decision.auto_driving<<" removed_person="<<removed;
  cv::rectangle(image,{0,0},{1280,29},{0,0,0},-1);
  cv::putText(image,label.str(),{8,21},cv::FONT_HERSHEY_SIMPLEX,.46,{255,255,255},1);
}
QJsonArray runtimeLibraries() {
  QFile maps("/proc/self/maps");QJsonArray out;std::set<QString> paths;
  if(maps.open(QIODevice::ReadOnly))for(const auto &row:maps.readAll().split('\n')){
    if(!row.contains("librknnrt")&&!row.contains("librga"))continue;
    const int slash=row.indexOf('/');if(slash>=0)paths.insert(QString::fromUtf8(row.mid(slash).trimmed()));
  }
  for(const auto &path:paths)out.append(fileRecord(path));
  return out;
}
struct SeenTrack { double last_real=0;unsigned predicted_samples=0;int class_id=-1; };
int run(Options o) {
  const auto run_start=Clock::now();cv::setNumThreads(1);
  cv::Mat still=cv::imread(o.input,cv::IMREAD_COLOR);const bool is_static=!still.empty();
  cv::VideoCapture capture;double source_fps=0,declared_frame_count=1;
  if(!is_static){capture.open(o.input);if(!capture.isOpened())throw std::runtime_error("cannot open real media input");
    source_fps=capture.get(cv::CAP_PROP_FPS);declared_frame_count=capture.get(cv::CAP_PROP_FRAME_COUNT);
    if(!std::isfinite(source_fps)||source_fps<=0||source_fps>1000)throw std::runtime_error("video lacks a usable FPS media clock");}
  if(is_static)for(const auto &c:o.commands)if(c.at>0)throw std::runtime_error("static commands must use the original single observation time, without positive time prefixes");
  const auto mode=o.drop_every?"controlled-drop-person-every-"+std::to_string(o.drop_every):"natural";
  const std::filesystem::path output=std::filesystem::path(o.output)/mode;
  if(!o.overwrite&&(std::filesystem::exists(output/"frames.jsonl")||std::filesystem::exists(output/"summary.json")))throw std::runtime_error("diagnostic mode directory already has results; choose a new output or --overwrite");
  std::filesystem::create_directories(output/"overlays");
  if(o.overwrite)for(const auto &entry:std::filesystem::directory_iterator(output/"overlays"))
    if(entry.is_regular_file()&&entry.path().extension()==".jpg")std::filesystem::remove(entry.path());
  std::ofstream frames(output/"frames.jsonl"),commands(output/"commands.jsonl"),csv(output/"frames.csv");
  if(!frames||!commands||!csv)throw std::runtime_error("cannot create diagnostic output files");
  csv<<"sequence,source_frame_index,media_s,static_image,clock_source,raw_person,raw_vehicle,raw_roadblock,raw_light_red,raw_light_green,roi_red,roi_green,after_dropout_person,controlled_person_removed,tracked_real,predicted,predicted_person,max_track_age_s,traffic_status,vehicle_status,roadblock_status,signal_status,raw_signal,decision_light,left_type,right_type,left_valid,right_valid,left_conf,right_conf,stop_required,auto_driving,brake_now,alarm_red_run,alarm_illegal_lane_change,alarm_collision,lane_change_allowed,arrow,led_mask,manual_stop,force_drive,force_lane_change,manual_nav,decision_speed_kmh,scene_ms,signal_ms,lane_ms,process_ms,command_count,full_role,signal_role,crosswalk_status,has_crosswalk,person_on_crosswalk\n";
  const QString manifest=qEnvironmentVariable("ADAS_MODEL_MANIFEST").isEmpty()?qs(o.root)+"/models/adas/manifest.json":qEnvironmentVariable("ADAS_MODEL_MANIFEST");
  NeuralBundleSpec spec;QString manifest_error;const bool manifest_ok=read_neural_manifest(manifest,spec,manifest_error);
  NeuralModels models;const auto init_start=Clock::now();const bool initialized=models.initialize(qs(o.root));const double init_ms=elapsed(init_start);
  const auto full_role=SceneModelRole::Vehicle,signal_role=SceneModelRole::Traffic;
  const Roi full_crop{0,0,1,1};
  const auto full_model=modelRecord(spec.vehicle,full_role);
  const auto signal_model=modelRecord(spec.traffic,signal_role);
  QJsonObject model_meta{{"manifest",fileRecord(manifest)},{"manifest_valid",manifest_ok},{"manifest_error",manifest_error},
    {"bundle",spec.version},{"initialized",initialized},{"description",models.description()},{"initialization_ms",num(init_ms)},
    {"model_profile","dual640"},{"lane_model_used",false},
    {"roles",QJsonObject{{"vehicle",full_model},{"traffic",signal_model}}},
    {"execution_order",QJsonArray{qs(roleName(full_role)),qs(roleName(signal_role))}},
    {"model_letterbox_padding_rgb",QJsonArray{0,0,0}},
    {"score_threshold",.25},{"nms_iou_threshold",.45},{"threshold_source","src/inference/neural_contract.cpp; unchanged"},
    {"postprocess_policy","Best class over all trained channels, then declared class selection; exact class-wise NMS; >300 survivors remain Failed, no top-K success"},
    {"runtime_libraries",runtimeLibraries()}};
  GroundCalibration calibration;if(!o.calibration.empty()){
    std::string error;if(!calibration.load(o.calibration,1280,720,&error))throw std::runtime_error("calibration rejected: "+error);}
  RoadGeometry geometry;ViewTracker tracker;AdasEngine engine;engine.reset(1);FramePool pool(2);EgoState ego;
  engine.required_obstacle_components=ObjectObservation::Combined;engine.separate_crosswalk_source=true;
  ego.input_layout=Layout::Single;ego.speed_source=SpeedSource::Simulation;ego.manual_speed_kmh=float(o.speed_kmh);ego.intersection_mode=o.intersection;
  const std::array<Roi,3> light_rois{{o.signal_roi,o.signal_roi,o.signal_roi}};
  const QJsonArray signal_roi_json{num(o.signal_roi.x),num(o.signal_roi.y),num(o.signal_roi.w),num(o.signal_roi.h)};
  Counts total_raw,total_roi,lane_counts,inference_errors;QJsonArray recoveries,overlay_paths,command_results;
  RoleStats full_stats,signal_stats;
  std::set<std::string> present;std::set<int64_t> ids;std::map<int64_t,SeenTrack> track_history;
  unsigned processed=0,source_index=0,removed_total=0,predictions_total=0,real_total=0,unassigned_total=0,failed_frames=0,command_count=0;
  double next_sample=0,last_media=-1,sum_process=0,max_process=0,first_media=0;AdasDecision last_decision;
  bool reached_eof=false;QJsonObject first_mapping;
  while(true){
    const auto decode_start=Clock::now();cv::Mat original;double media=0;std::string clock_source="static_image";
    if(is_static){if(processed)break;original=still;}
    else {
      if(!capture.read(original)){reached_eof=true;break;}
      const double timestamp=capture.get(cv::CAP_PROP_POS_MSEC)/1000.;
      media=double(source_index)/source_fps;clock_source="source_frame_index/fps";
      if(std::isfinite(timestamp)&&timestamp>=0&&(source_index==0||timestamp>last_media+1e-9)){
        media=timestamp;clock_source="CAP_PROP_POS_MSEC";}
      if(media+1e-8<next_sample){++source_index;continue;}
      if(processed&&media<=last_media)throw std::runtime_error("media timestamp did not advance; refusing duplicate-time confirmations");
      // Keep sampling targets anchored to the media clock; rounding each
      // accepted timestamp would turn requested 10 Hz on 25 fps into 8.33 Hz.
      next_sample+=1./std::min(o.sample_fps,source_fps);
    }
    const double decode_ms=elapsed(decode_start);const auto frame_start=Clock::now();
    const Stamp stamp{1,uint64_t(processed+1),media,media,is_static};QJsonObject mapping;
    cv::Mat image=canvas(original,mapping),packed;cv::cvtColor(image,packed,cv::COLOR_BGR2BGR565);
    auto packet=pool.copy(packed.data,packed.cols,packed.rows,int(packed.step),stamp,Layout::Single);
    if(!packet)throw std::runtime_error("cannot create immutable RGB565 packet");
    const auto front=view_rect(packet.layout,ViewId::Front,packet.width,packet.height);
    if(ego.input_layout!=Layout::Single||packet.layout!=Layout::Single||
       front.x!=0||front.y!=0||front.w!=1280||front.h!=720)
      throw std::runtime_error("diagnostic requires the complete single HDMI front canvas");
    cv::cvtColor(packed,image,cv::COLOR_BGR5652BGR);const double prepare_ms=elapsed(frame_start);
    // Each source frame executes V,T once; the complete HDMI canvas is used by
    // both schema3 models. Light ROIs select evidence after full-frame decoding.
    auto full=models.scene(packet,ViewId::Front,light_rois,full_crop,full_role);
    auto signal=models.scene(packet,ViewId::Front,light_rois,full_crop,signal_role);
    full_stats.add(full);signal_stats.add(signal);
    const auto raw_full=rawJson(full.vision),raw_roi=rawJson(signal.vision);
    const auto raw_counts=modelCounts(full.vision),roi_counts=modelCounts(signal.vision);
    addCounts(total_raw,raw_counts);addCounts(total_roi,roi_counts);
    for(const auto &c:raw_counts)if(c.second)present.insert(c.first);
    for(const auto &c:roi_counts)if(c.second)present.insert(c.first);
    auto lane=geometry.analyze_lane(image,stamp);
    if(lane.components!=GeometryObservation::Lane||lane.nav.detected||lane.speed.valid)throw std::runtime_error("CPU lane producer violated Lane-only contract");
    const auto track_start=Clock::now();auto &objects=full.vision.observation;objects.signal_observation=false;
    const bool controlled=o.drop_every&&stamp.sequence%o.drop_every==0;unsigned removed=0;
    if(controlled){
      auto &list=objects.obstacles;const auto previous=list.size();list.erase(std::remove_if(list.begin(),list.end(),[](const auto&b){return b.class_id==1;}),list.end());removed=unsigned(previous-list.size());
      auto &overlays=full.vision.overlays;overlays.erase(std::remove_if(overlays.begin(),overlays.end(),[](const auto&b){return b.class_id==1;}),overlays.end());
      if(removed)objects.person_on_zebra=false;
    }
    unsigned after_person=0;for(const auto &b:objects.obstacles)if(b.class_id==1)++after_person;
    const bool tracked=tracker.update(objects);unsigned real=0,predicted=0,predicted_person=0,unassigned=0;double max_age=0;QJsonArray frame_recoveries;
    for(auto &b:objects.obstacles){
      if(b.predicted){b.distance_m=-1;++predicted;if(b.class_id==1)++predicted_person;}
      else {++real;if(calibration.is_compatible(objects.width,objects.height))b.distance_m=float(calibration.distance(b.box));}
      max_age=std::max(max_age,media-b.last_observed_s);
      if(b.track_id<0){if(!b.predicted)++unassigned;continue;}
      ids.insert(b.track_id);
      auto &history=track_history[b.track_id];
      if(b.predicted)++history.predicted_samples;
      else {
        if(history.predicted_samples){QJsonObject event{{"track_id",QString::number(b.track_id)},{"class",qs(kind(b.class_id))},
          {"last_real_s",num(history.last_real)},{"recovered_s",num(media)},{"gap_s",num(media-history.last_real)},
          {"predicted_samples",int(history.predicted_samples)},{"controlled_dropout",o.drop_every>0},
          {"interpretation","same tracker ID after coasting; true identity is not annotated"}};
          recoveries.append(event);frame_recoveries.append(event);}
        history.last_real=media;history.predicted_samples=0;history.class_id=b.class_id;
      }
    }
    sync_tracking_overlays(full.vision,media);const double tracking_ms=elapsed(track_start);
    SignalObservation lamp;lamp.stamp=stamp;lamp.status=signal.vision.observation.signal_status;lamp.light=signal.vision.observation.light;
    lamp.has_lane_lights=signal.vision.observation.has_lane_lights;lamp.lane_lights=signal.vision.observation.lane_lights;
    if(lamp.status==Status::Valid&&lamp.light==LightState::Red)present.insert("signal-red");
    if(lamp.status==Status::Valid&&lamp.light==LightState::Green)present.insert("signal-green");
    const bool ingested_objects=engine.ingest(objects,media);
    auto crosswalk=signal.vision.observation;
    crosswalk.components=ObjectObservation::Crosswalk;crosswalk.signal_observation=false;
    crosswalk.obstacles.clear();
    const bool ingested_crosswalk=engine.ingest(crosswalk,media);
    const bool ingested_signal=engine.ingest(lamp,media),ingested_lane=engine.ingest(lane,media);
    const auto engine_start=Clock::now();auto decision=engine.evaluate(ego,media);QJsonArray frame_commands;
    auto logCommand=[&](const std::string &name,double requested){
      QJsonObject entry{{"sequence",num(stamp.sequence)},{"media_s",num(media)},{"requested_s",num(requested)},
        {"command",qs(name)},{"same_observation_reevaluation",true},{"ego",egoJson(ego)},{"decision",decisionJson(decision)}};
      jsonLine(commands,entry);frame_commands.append(entry);command_results.append(entry);
    };
    logCommand("baseline",media);
    for(auto &c:o.commands)if(!c.executed&&c.at<=media+1e-8){
      const auto requested_action=action(c.name);apply_manual_action(ego,requested_action);
      // Every explicit direction command is a new operator request, including
      // repeated keys that happen to leave the direction enum unchanged.
      if(requested_action==ManualAction::NavigateLeft||requested_action==ManualAction::NavigateStraight||
         requested_action==ManualAction::NavigateRight||requested_action==ManualAction::SignalLeft||
         requested_action==ManualAction::SignalRight||requested_action==ManualAction::ForceChangeLeft||
         requested_action==ManualAction::ForceChangeRight||requested_action==ManualAction::ToggleForceLeft||
         requested_action==ManualAction::ToggleForceRight||requested_action==ManualAction::CycleNavigation)
        ego.lane_change_request_s=media;
      c.executed=true;++command_count;decision=engine.evaluate(ego,media);logCommand(c.name,c.at);
    }
    const double engine_ms=elapsed(engine_start),process_ms=elapsed(frame_start);last_decision=decision;
    const auto &l=lane.lane;
    if(l.left_valid){++lane_counts["left-"+line(l.left_type)];present.insert("lane");present.insert("left-"+line(l.left_type));}
    if(l.right_valid){++lane_counts["right-"+line(l.right_type)];present.insert("lane");present.insert("right-"+line(l.right_type));}
    QJsonArray lane_lights;for(auto color:lamp.lane_lights)lane_lights.append(qs(light(color)));
    QJsonObject row{{"stamp",stampJson(stamp)},{"source_frame_index",int(source_index)},{"clock_source",qs(clock_source)},
      {"input_layout","HDMI_SINGLE"},{"front_view_canvas_xywh",QJsonArray{front.x,front.y,front.w,front.h}},
      {"full_inference",inferenceJson(full)},{"signal_inference",inferenceJson(signal)},
      {"execution_roles",QJsonArray{qs(roleName(full_role)),qs(roleName(signal_role))}},
      {"role_inference",QJsonObject{{"vehicle",inferenceJson(full)},{"traffic",inferenceJson(signal)}}},
      {"mapping",mapping},{"model_ready",full.vision.model_ready},{"signal_model_ready",signal.vision.model_ready},
      {"raw_full_detections",raw_full},{"raw_signal_roi_detections",raw_roi},{"raw_full_counts",countsJson(raw_counts)},
      {"raw_traffic_detections",raw_roi},
      {"raw_signal_roi_counts",countsJson(roi_counts)},{"statuses",statuses(objects)},
      {"full_model_error",qs(full.vision.error_message)},{"traffic_statuses",statuses(signal.vision.observation)},
      {"crosswalk_observation",QJsonObject{{"stamp",stampJson(signal.vision.observation.stamp)},
        {"status",qs(status(signal.vision.observation.crosswalk_status))},
        {"has_crosswalk",signal.vision.observation.has_zebra},{"person_on_crosswalk",signal.vision.observation.person_on_zebra}}},
      {"signal_status",qs(status(lamp.status))},{"signal_model_error",qs(signal.vision.error_message)},
      {"signal_raw",qs(light(lamp.light))},{"full_scene_raw_light",qs(light(objects.light))},
      {"lane_lights_raw",lane_lights},{"signal_crop",QJsonArray{num(full_crop.x),num(full_crop.y),num(full_crop.w),num(full_crop.h)}},
      {"three_lane_light_rois",QJsonArray{signal_roi_json,signal_roi_json,signal_roi_json}},
      {"lane",laneJson(lane)},{"tracker_updated",tracked},{"tracked_obstacles",trackedJson(objects)},
      {"tracked_real_count",int(real)},{"predicted_count",int(predicted)},{"predicted_person_count",int(predicted_person)},
      {"assigned_real_count",int(real-unassigned)},{"unassigned_real_count",int(unassigned)},
      {"maximum_track_age_s",num(max_age)},{"controlled_dropout",o.drop_every>0},{"controlled_dropout_applied",controlled},
      {"person_removed_count",int(removed)},{"after_dropout_person_count",int(after_person)},{"same_id_recoveries",frame_recoveries},
      {"ingest",QJsonObject{{"objects",ingested_objects},{"crosswalk",ingested_crosswalk},{"signal",ingested_signal},{"lane",ingested_lane},
        {"object_observation_count",1},{"signal_observation_count",1},{"lane_observation_count",1},
        {"crosswalk_observation_count",1},{"required_obstacle_components",int(engine.required_obstacle_components)},
        {"separate_crosswalk_source",engine.separate_crosswalk_source},{"traffic_updates_primary_tracker",false},
        {"full_scene_signal_confirmation",false},{"navigation_observation_count",0},{"visual_speed_observation_count",0}}},
      {"ego",egoJson(ego)},{"decision",decisionJson(decision)},{"command_evaluations",frame_commands},
      {"timings_ms",QJsonObject{{"decode",num(decode_ms)},{"prepare",num(prepare_ms)},
        {"scene",num(objects.infer_ms)},{"signal",num(signal.vision.observation.infer_ms)},
        {"lane",num(lane.process_ms)},{"tracking",num(tracking_ms)},{"engine",num(engine_ms)},{"analysis_total",num(process_ms)}}}};
    jsonLine(frames,row);
    csv<<stamp.sequence<<','<<source_index<<','<<std::setprecision(10)<<media<<','<<is_static<<','<<clock_source<<','
      <<count(raw_counts,"person")<<','<<count(raw_counts,"vehicle")<<','<<count(raw_counts,"roadblock")<<','<<count(raw_counts,"light_red")<<','<<count(raw_counts,"light_green")<<','
      <<count(roi_counts,"light_red")<<','<<count(roi_counts,"light_green")<<','<<after_person<<','<<removed<<','<<real<<','<<predicted<<','<<predicted_person<<','<<max_age<<','
      <<status(objects.traffic_status)<<','<<status(objects.vehicle_status)<<','<<status(objects.roadblock_status)<<','<<status(lamp.status)<<','<<light(lamp.light)<<','<<light(decision.light)<<','
      <<line(l.left_type)<<','<<line(l.right_type)<<','<<l.left_valid<<','<<l.right_valid<<','<<l.left_conf<<','<<l.right_conf<<','
      <<decision.stop_required<<','<<decision.auto_driving<<','<<decision.brake_now<<','<<decision.alarm_red_run<<','<<decision.alarm_illegal_lane_change<<','<<decision.alarm_collision<<','
      <<decision.lane_change_allowed<<','<<direction(decision.arrow)<<','<<int(decision.led_mask)<<','<<ego.manual_stop<<','<<ego.force_drive<<','<<turn(ego.force_lane_change)<<','
      <<direction(ego.manual_nav)<<','<<decision.speed_kmh<<','<<objects.infer_ms<<','<<signal.vision.observation.infer_ms<<','<<lane.process_ms<<','<<process_ms<<','<<frame_commands.size()-1<<','
      <<roleName(full_role)<<','<<roleName(signal_role)<<','<<status(signal.vision.observation.crosswalk_status)<<','
      <<signal.vision.observation.has_zebra<<','<<signal.vision.observation.person_on_zebra<<'\n';
    if(!csv)throw std::runtime_error("cannot write diagnostic CSV");
    frames.flush();commands.flush();csv.flush();
    ++processed;if(processed==1){first_media=media;first_mapping=mapping;}
    removed_total+=removed;predictions_total+=predicted;real_total+=real;unassigned_total+=unassigned;sum_process+=process_ms;max_process=std::max(max_process,process_ms);last_media=media;
    if(!full.vision.model_ready||!signal.vision.model_ready||lane.status!=Status::Valid)++failed_frames;
    if(!full.vision.model_ready)++inference_errors[roleName(full_role)+":full:"+objects.error];
    if(!signal.vision.model_ready)++inference_errors[roleName(signal_role)+":signal:"+signal.vision.error_message];
    if(processed==1||(o.overlay_every&&processed%o.overlay_every==0)){
      drawOverlay(image,full.vision,signal.vision,lane,decision,processed,media,removed);
      std::ostringstream filename;filename<<"overlays/frame-"<<std::setw(5)<<std::setfill('0')<<processed<<".jpg";
      if(!cv::imwrite((output/filename.str()).string(),image))throw std::runtime_error("cannot save sparse overlay");
      overlay_paths.append(qs(filename.str()));
    }
    std::cout<<mode<<" seq="<<processed<<" source="<<source_index<<" t="<<media<<" person="<<count(raw_counts,"person")
      <<" predicted="<<predicted<<" lanes="<<line(l.left_type)<<'/'<<line(l.right_type)<<" signal="<<light(lamp.light)<<" ms="<<process_ms<<std::endl;
    if(is_static||!initialized||(!o.continue_on_error&&failed_frames)||(o.max_frames&&processed>=o.max_frames))break;
    ++source_index;
  }
  QJsonArray expectations,pending;bool presence_pass=true;
  for(const auto &e:o.expect){const bool found=present.count(e);expectations.append(QJsonObject{{"expected_scene_category",qs(e)},{"observed_at_least_once",found},
    {"evaluation_type","coarse_presence_only; no instance ground truth or accuracy metric"}});presence_pass=presence_pass&&found;}
  for(const auto &c:o.commands)if(!c.executed)pending.append(QJsonObject{{"command",qs(c.name)},{"requested_s",num(c.at)}});
  QJsonArray core_objects,source_files;
  for(const char *name:{"adas_engine","ground_calibration","road_geometry","neural_contract","neural_models","rga_preprocessor"})
    if(!o.object_dir.empty())core_objects.append(fileRecord(qs(o.object_dir)+"/"+name+".o"));
  for(const char *path:{"tools/adas_scene_diagnostic.cpp","tools/run_scene_diagnostics.sh","src/core/adas_types.h","src/core/view_tracker.h","src/core/tracked_overlays.h","src/core/tracking_evidence.h","src/core/manual_controls.h","src/core/road_geometry.cpp","src/core/adas_engine.cpp","src/inference/neural_contract.h","src/inference/neural_contract.cpp","src/inference/neural_models.h","src/inference/neural_models.cpp","src/inference/neural_scene_adapter.h"})source_files.append(fileRecord(qs(o.root)+"/"+path));
  const bool infrastructure_ok=initialized&&processed>0&&failed_frames==0;
  QJsonObject summary{{"schema_version",2},{"created_utc",QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)},
    {"input_layout","HDMI_SINGLE"},{"front_view_canvas_xywh",QJsonArray{0,0,1280,720}},
    {"status",!infrastructure_ok?"infrastructure_or_inference_failed":!presence_pass?"scene_presence_failed":"diagnostic_completed"},
    {"input",fileRecord(qs(o.input))},{"output_directory",qs(output.string())},
    {"static_image",is_static},{"static_ingest_count",is_static?int(processed):0},{"processed_frames",int(processed)},
    {"source_fps",num(source_fps)},{"declared_source_frame_count",num(declared_frame_count)},
    {"requested_sample_fps",num(o.sample_fps)},{"max_frames",int(o.max_frames)},
    {"effective_sample_fps",num(processed>1?double(processed-1)/(last_media-first_media):0)},
    {"first_media_s",num(first_media)},{"last_media_s",num(last_media)},{"reached_video_eof",reached_eof},
    {"frame_limit_reached",!is_static&&o.max_frames&&processed>=o.max_frames},{"first_frame_mapping",first_mapping},
    {"controlled_dropout",o.drop_every>0},{"drop_person_every",int(o.drop_every)},{"persons_removed",int(removed_total)},
    {"signal_roi_normalized_canvas",signal_roi_json},{"three_lane_light_rois",QJsonArray{signal_roi_json,signal_roi_json,signal_roi_json}},
    {"signal_model_crop",QJsonArray{num(full_crop.x),num(full_crop.y),num(full_crop.w),num(full_crop.h)}},
    {"model_profile","dual640"},{"execution_roles_per_sample",QJsonArray{qs(roleName(full_role)),qs(roleName(signal_role))}},
    {"required_obstacle_components",int(engine.required_obstacle_components)},{"separate_crosswalk_source",engine.separate_crosswalk_source},
    {"role_statistics",QJsonObject{{"vehicle",full_stats.json()},{"traffic",signal_stats.json()}}},
    {"raw_full_class_counts",countsJson(total_raw)},{"raw_signal_roi_class_counts",countsJson(total_roi)},
    {"raw_traffic_class_counts",countsJson(total_roi)},
    {"lane_type_frame_counts",countsJson(lane_counts)},{"tracked_real_observations",int(real_total)},
    {"predicted_observations",int(predictions_total)},{"unique_tracker_ids",int(ids.size())},
    {"assigned_real_observations",int(real_total-unassigned_total)},{"unassigned_real_observations",int(unassigned_total)},
    {"same_id_recovery_events",recoveries},{"natural_short_dropout_same_id_recoveries",o.drop_every?0:recoveries.size()},
    {"controlled_dropout_same_id_recoveries",o.drop_every?recoveries.size():0},
    {"inference_failed_frames",int(failed_frames)},{"models",model_meta},
    {"continue_on_inference_error",o.continue_on_error},{"inference_error_counts",countsJson(inference_errors)},
    {"navigation_source","manual_controls_only"},{"speed_source","explicit_simulation"},
    {"calibration",o.calibration.empty()?QJsonObject():fileRecord(qs(o.calibration))},
    {"distance_policy","Only real observations may use supplied calibration; predictions always distance_m=-1"},
    {"command_count",int(command_count)},{"pending_commands",pending},{"command_evaluations",command_results},
    {"final_ego",egoJson(ego)},{"final_decision",decisionJson(last_decision)},{"expectation_presence_checks",expectations},
    {"presence_checks_passed",presence_pass},{"annotation_level","scene_only"},{"accuracy_metrics_computed",false},
    {"limitations",QJsonArray{"No per-instance boxes, lane geometry, true identities, mAP or MOTA ground truth.",
      "Same-ID recovery counts describe tracker behavior after coasting, not proven identity correctness.",
      "Controlled deletions are artificial measurement loss and must not be described as natural model misses.",
      "Model classes not detected remain zero; thresholds are unchanged; photos do not provide motion evidence."}},
    {"processing_ms_mean",num(processed?sum_process/processed:0)},{"processing_ms_max",num(max_process)},
    {"wall_elapsed_ms",num(elapsed(run_start))},{"sparse_overlays",overlay_paths},
    {"binary",fileRecord(QCoreApplication::applicationFilePath())},{"core_objects",core_objects},{"source_files",source_files},
    {"production_input","PC HDMI -> FPGA -> ARM; this standalone executable does not alter production ingestion"}};
  writeJson(output/"summary.json",summary);
  std::cout<<"summary="<<(output/"summary.json")<<" status="<<summary.value("status").toString().toStdString()<<std::endl;
  return !infrastructure_ok?3:!presence_pass?4:0;
}
} // namespace
int main(int argc,char **argv) {
  QCoreApplication app(argc,argv);
  try { return run(parse(argc,argv)); }
  catch(const std::exception &e){std::cerr<<"adas_scene_diagnostic: "<<e.what()<<'\n';return 2;}
}
