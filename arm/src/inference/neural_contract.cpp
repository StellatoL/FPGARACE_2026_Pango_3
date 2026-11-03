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
#include <limits>
#include <set>
#include <stdexcept>
namespace adas {
namespace {
void require(bool value,const char *message){if(!value)throw std::runtime_error(message);}
bool scene_dimensions(int w,int h){return w==640&&h==640;}
bool scene_class_count(size_t n){return n==6||n==80;}
bool finite(const std::vector<float> &values){
  return std::all_of(values.begin(),values.end(),[](float value){return std::isfinite(value);});
}
const std::vector<std::string> traffic_classes{
    "crosswalk","person","light_red","light_green","light_black","light_yellow"};
const std::vector<std::string> vehicle_classes{
    "person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard",
    "sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
    "donut","cake","chair","couch","potted plant","bed","dining table","toilet",
    "tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush"};
const std::array<std::array<float,6>,3> traffic_anchors{{
    {5.64062f,12.39062f,7.85547f,18.5f,10.71875f,28.14062f},
    {63.5f,9.07812f,15.92188f,41.3125f,25.48438f,69.9375f},
    {218.375f,16.32812f,49.90625f,100.3125f,612.5f,90.f}}};

NeuralModelSpec read_model(const QJsonObject &obj,const QDir &base,SceneModelRole role) {
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
  require(input.value("layout").toString()=="NHWC"&&input.value("format").toString()=="RGB8","model input must be NHWC RGB8");
  const auto mean=input.value("mean").toArray(),stddev=input.value("std").toArray();
  require(mean.size()==3&&stddev.size()==3,"input normalization is required");
  for(int i=0;i<3;++i)require(mean[i].toDouble(-1)==0&&stddev[i].toDouble(-1)==255,"input normalization must be mean0/std255");
  s.width=input.value("width").toInt();s.height=input.value("height").toInt();
  require(scene_dimensions(s.width,s.height),"unsupported model input dimensions");
  require(obj.value("quantization").toString()=="int8","only INT8 ADAS models are accepted");
  s.output=obj.value("output").toString();
  require(s.output==(role==SceneModelRole::Traffic?"yolov5_raw_nchw_v1":"yolov5_rk_sigmoid_nchw_v1"),"model role output contract mismatch");
  s.runtime_api_prefix=obj.value("runtime_api_prefix").toString();require(!s.runtime_api_prefix.isEmpty(),"runtime_api_prefix is required");
  std::set<std::string> names;
  for(const auto &v:obj.value("classes").toArray()){
    auto name=v.toString().toStdString();require(!name.empty()&&names.insert(name).second,"empty/duplicate scene class");s.classes.push_back(name);
  }
  require(s.classes==(role==SceneModelRole::Traffic?traffic_classes:vehicle_classes),"model role class order mismatch");
  const auto anchors=obj.value("anchors").toArray();require(anchors.size()==3,"three anchor heads required");
  for(int h=0;h<3;++h){
    const auto row=anchors[h].toArray();require(row.size()==6,"six anchor dimensions required per head");
    for(int a=0;a<6;++a){s.anchors[h][a]=float(row[a].toDouble());require(std::isfinite(s.anchors[h][a])&&s.anchors[h][a]>0&&s.anchors[h][a]<=1280,"invalid anchor dimension");}
  }
  if(role==SceneModelRole::Traffic)require(s.anchors==traffic_anchors,"traffic anchors mismatch");
  return s;
}
}
bool read_neural_manifest(const QString &path,NeuralBundleSpec &spec,QString &error){
  spec={};error.clear();
  try{
    QFile file(path);require(file.open(QIODevice::ReadOnly),"ADAS model bundle is not configured: manifest missing");
    require(file.size()<=65536,"manifest too large");QJsonParseError parse;
    const auto doc=QJsonDocument::fromJson(file.readAll(),&parse);
    require(parse.error==QJsonParseError::NoError&&doc.isObject(),"invalid manifest JSON");
    const auto obj=doc.object();require(obj.value("schema").toInt()==3&&obj.value("target").toString()=="rk3568","manifest schema/target mismatch");
    NeuralBundleSpec result;result.version=obj.value("version").toString();require(!result.version.isEmpty(),"bundle version is required");
    require(!obj.value("toolkit_version").toString().isEmpty(),"toolkit version is required");
    require(!obj.contains("lane")||obj.value("lane").isNull(),"neural lane models are not supported");
    require(obj.value("traffic").isObject()&&obj.value("vehicle").isObject(),"schema3 requires traffic and vehicle models");
    const auto base=QFileInfo(path).absoluteDir();
    result.traffic=read_model(obj.value("traffic").toObject(),base,SceneModelRole::Traffic);
    result.vehicle=read_model(obj.value("vehicle").toObject(),base,SceneModelRole::Vehicle);
    require(result.traffic.path!=result.vehicle.path,"traffic and vehicle models must use distinct files");
    result.vehicle.selected_classes={0,1,2,3,5,7};result.dual=true;spec=std::move(result);return true;
  }catch(const std::exception&e){error=QString::fromUtf8(e.what());return false;}
}
bool validate_neural_outputs(const std::vector<NeuralTensorSpec>&t,size_t classes,int width,int height,std::string&error){
  error.clear();
  if(t.size()!=3){error="output count mismatch";return false;}
  if(!scene_dimensions(width,height)||!scene_class_count(classes)){error="model input dimensions/classes invalid";return false;}
  for(size_t i=0;i<t.size();++i){
    const unsigned side=unsigned(width)/(8u<<i);
    if(t[i].shape!=std::vector<unsigned>{1,unsigned(3*(classes+5)),side,side}){error="scene head shape/order mismatch";return false;}
  }
  return true;
}
bool decode_neural_scene(const std::vector<std::vector<float>> &heads,
    const NeuralModelSpec &spec, std::vector<NeuralDetection> &out,
    std::string &error, SceneDecodeStats *stats) {
  out.clear();
  error.clear();
  SceneDecodeStats local_stats;
  auto &counts = stats ? *stats : local_stats;
  counts = {};
  const auto fail = [&](const char *message) {
    out.clear();
    error = message;
    return false;
  };
  if (!scene_dimensions(spec.width, spec.height)) return fail("scene input dimensions invalid");
  const auto classes = spec.classes.size();
  if (heads.size() != 3 || !scene_class_count(classes)) return fail("scene tensor count/classes invalid");
  std::vector<bool> selected(classes, spec.selected_classes.empty());
  for (const int id : spec.selected_classes) {
    if (id < 0 || size_t(id) >= classes || selected[size_t(id)]) return fail("scene selected classes invalid");
    selected[size_t(id)] = true;
  }
  const bool probabilities = spec.output == "yolov5_rk_sigmoid_nchw_v1";
  if (!probabilities && spec.output != "yolov5_raw_nchw_v1") return fail("scene output contract invalid");

  // Sigmoid implementations can overshoot by a few float ULPs. Validate every
  // head before decoding; clamp only these bounded activation roundoff errors.
  constexpr float probability_tolerance = 4 * std::numeric_limits<float>::epsilon();
  size_t max_candidates = 0;
  for (size_t h = 0; h < heads.size(); ++h) {
    const unsigned side = unsigned(spec.width) / (8u << h), grid = side * side;
    const auto &head = heads[h];
    if (head.size() != 3 * (classes + 5) * grid || !finite(head))
      return fail("scene tensor shape/values invalid");
    if (probabilities) {
      for (const float value : head) {
        if (value < -probability_tolerance || value > 1.f + probability_tolerance)
          return fail("RK sigmoid output outside probability range");
        if (value < 0 || value > 1) ++counts.probability_clamps;
      }
    }
    max_candidates += 3 * grid;
  }
  const auto sigmoid = [](float x) {
    return x >= 0 ? 1.f / (1.f + std::exp(-x)) : std::exp(x) / (1.f + std::exp(x));
  };
  const auto activate = [&](float x) { return probabilities ? std::clamp(x, 0.f, 1.f) : sigmoid(x); };

  // Tensor dimensions bound storage to 25,200 candidates. Retaining them all
  // avoids discarding boxes before exact NMS.
  std::vector<NeuralDetection> candidates;
  candidates.reserve(max_candidates);
  for (size_t h = 0; h < heads.size(); ++h) {
    const unsigned stride = 8u << h, side = unsigned(spec.width) / stride;
    const unsigned grid = side * side, props = unsigned(classes + 5);
    const auto &head = heads[h];
    for (unsigned a = 0; a < 3; ++a) for (unsigned cell = 0; cell < grid; ++cell) {
      const unsigned base = a * props * grid + cell;
      const float obj = activate(head[base + 4 * grid]);
      if (obj < .25f) continue;
      int best = 0;
      float raw = head[base + 5 * grid];
      for (unsigned c = 1; c < classes; ++c) {
        if (head[base + (5 + c) * grid] > raw) {
          raw = head[base + (5 + c) * grid];
          best = int(c);
        }
      }
      if (!selected[size_t(best)]) continue;
      const float score = obj * activate(raw);
      if (score < .25f) continue;
      const float x = (activate(head[base]) * 2 - .5f + cell % side) * stride;
      const float y = (activate(head[base + grid]) * 2 - .5f + cell / side) * stride;
      const float sw = activate(head[base + 2 * grid]) * 2, sh = activate(head[base + 3 * grid]) * 2;
      const float w = sw * sw * spec.anchors[h][a * 2], hh = sh * sh * spec.anchors[h][a * 2 + 1];
      if (!std::isfinite(w) || !std::isfinite(hh) || w <= 0 || hh <= 0)
        return fail("scene anchor/box invalid");
      candidates.push_back({best, {x - w / 2, y - hh / 2, x + w / 2, y + hh / 2}, score});
    }
  }
  counts.candidate_count = candidates.size();
  // Equal scores keep head/anchor/cell order across platforms and repeated runs.
  std::stable_sort(candidates.begin(), candidates.end(), [](const auto &a, const auto &b) {
    return a.confidence > b.confidence;
  });
  for (const auto &candidate : candidates) {
    bool suppressed = false;
    const auto &a = candidate.box;
    // Only earlier accepted boxes can suppress this candidate. Checking them
    // here avoids visiting the low-score tail once saturation is established.
    for (const auto &kept : out) {
      if (kept.class_id != candidate.class_id) continue;
      ++counts.nms_comparisons;
      const auto &b = kept.box;
      const float overlap = std::max(0.f, std::min(a.right, b.right) - std::max(a.left, b.left)) *
          std::max(0.f, std::min(a.bottom, b.bottom) - std::max(a.top, b.top));
      const float total = (a.right - a.left) * (a.bottom - a.top) +
          (b.right - b.left) * (b.bottom - b.top) - overlap;
      if (total > 0 && overlap / total > .45f) {
        suppressed = true;
        break;
      }
    }
    if (suppressed) continue;
    ++counts.nms_kept;
    // The 301st survivor proves output saturation. Fail with no partial results;
    // stopping here bounds NMS work without truncating a successful scene.
    if (counts.nms_kept > 300) return fail("scene output limit exceeded");
    out.push_back(candidate);
  }
  return true;
}
}
