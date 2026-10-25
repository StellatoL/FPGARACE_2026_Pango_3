#include "inference/neural_models.h"
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <cassert>
#include <iostream>
int main(int argc,char **argv) {
  QCoreApplication app(argc,argv);QTemporaryDir dir;
  const QByteArray bytes("status fixture, never submitted to RKNN");
  QFile weight(dir.filePath("scene.rknn"));assert(weight.open(QIODevice::WriteOnly));weight.write(bytes);weight.close();
  QJsonArray classes;for(const char *s:{"person","vehicle","crosswalk","light_red","light_green","cone","barrier","debris"})classes.append(s);
  QJsonObject scene{{"file","scene.rknn"},{"sha256",QString::fromLatin1(QCryptographicHash::hash(bytes,QCryptographicHash::Sha256).toHex())},
    {"quantization","int8"},{"runtime_api_prefix","2.3.2"},{"output","yolov5_rk_sigmoid_nchw_v1"},{"classes",classes},
    {"anchors",QJsonArray{QJsonArray{10,13,16,30,33,23},QJsonArray{30,61,62,45,59,119},QJsonArray{116,90,156,198,373,326}}},
    {"input",QJsonObject{{"width",640},{"height",640},{"layout","NHWC"},{"format","RGB8"},{"mean",QJsonArray{0,0,0}},{"std",QJsonArray{255,255,255}}}}};
  QJsonObject manifest{{"schema",2},{"target","rk3568"},{"version","fixture"},{"toolkit_version","fixture"},
    {"road_roi",QJsonArray{0,.42,1,.58}},{"scene",scene},{"lane",QJsonValue(QJsonValue::Null)}};
  QFile file(dir.filePath("manifest.json"));assert(file.open(QIODevice::WriteOnly));file.write(QJsonDocument(manifest).toJson());file.close();
  qputenv("ADAS_MODEL_MANIFEST",file.fileName().toUtf8());qputenv("PANGO_DISABLE_RGA","1");
  adas::NeuralModels models;assert(!models.initialize(dir.path()));
  adas::FramePacket packet;packet.width=640;packet.height=360;packet.stride=1280;packet.layout=adas::Layout::Single;
  auto objects=models.scene(packet,adas::ViewId::Front,{},{});
  assert(objects.vision.observation.traffic_status==adas::Status::Failed);
  auto lane=models.lane(packet);
  assert(lane.status==adas::Status::Missing); // Scene/RGA failure cannot configure a nonexistent lane model.
  assert(!lane.lane.left_valid&&!lane.lane.right_valid);
  std::cout<<"independent absent-lane status through RGA failure: PASS\n";
}
