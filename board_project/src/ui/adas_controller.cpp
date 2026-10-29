#include "core/manual_controls.h"
#include "core/tracking_evidence.h"
#include "core/collision_geometry.h"
#include "ui/adas_controller.h"
#include "ui/mainwindow.h"
#include "ui_mainwindow.h"
#include "hardware/pango_fun.h"
#include "inference/adas_pipeline.h"
#include "inference/neural_contract.h"
#include <QtWidgets>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QCryptographicHash>
#include <algorithm>
#include <cmath>

namespace {
QString project_root() {
  QDir dir(QCoreApplication::applicationDirPath());
  if (dir.exists("models")) return dir.absolutePath();
  if (dir.cdUp() && dir.exists("models")) return dir.absolutePath();
  return QDir::currentPath();
}
QString config_path() { return project_root()+"/config/adas.ini"; }
QString direction_text(adas::Direction dir) {
  switch(dir) { case adas::Direction::Left:return QStringLiteral("左");
  case adas::Direction::Right:return QStringLiteral("右");
  case adas::Direction::Straight:return QStringLiteral("直行");default:return QStringLiteral("停车"); }
}
QStringList alarm_messages(const adas::AdasDecision &d) {
  QStringList messages;
  if(d.brake_now)messages<<QStringLiteral("行人急刹");
  if(d.alarm_collision)messages<<QStringLiteral("碰撞危险");
  if(d.alarm_red_run)messages<<QStringLiteral("红灯强行驶");
  if(d.alarm_illegal_lane_change)messages<<QStringLiteral("实线强制变道");
  if(d.alarm_overspeed)messages<<QStringLiteral("超速警告");
  if(d.alarm_lane_departure)messages<<QStringLiteral("车道偏离 / 压实线");
  return messages;
}
QString motion_text(const adas::AdasDecision &d,const adas::EgoState &ego) {
  return !d.auto_driving?QStringLiteral("停车"):ego.force_drive?QStringLiteral("强制行驶"):QStringLiteral("模拟行驶");
}
QString navigation_text(const adas::EgoState &ego) {
  return ego.manual_nav_enabled?QStringLiteral("手动导航：")+direction_text(ego.manual_nav):QStringLiteral("手动导航：关闭");
}
QString input_fps_text(double fps) {
  return QStringLiteral("FPGA输入 %1 FPS").arg(fps,0,'f',1);
}
QGroupBox *group(const QString &title, QWidget *parent, QVBoxLayout **layout) {
  auto *box=new QGroupBox(title,parent); *layout=new QVBoxLayout(box);
  (*layout)->setContentsMargins(7,8,7,7); (*layout)->setSpacing(5); return box;
}
QLabel *wrapping_label(QWidget *parent) {
  auto *label=new QLabel(parent); label->setWordWrap(true);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse); return label;
}
// Native coordinates remain stable while the dialog image is letterboxed.
class CalibrationCanvas : public QWidget {
public:
  explicit CalibrationCanvas(QImage image,QWidget *parent=nullptr):QWidget(parent),image_(std::move(image)) {
    setMinimumSize(480,270); setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
  }
  std::vector<cv::Point2f> points;
protected:
  QRect image_rect() const {
    auto size=image_.size(); size.scale(this->size(),Qt::KeepAspectRatio);
    return QRect(QPoint((width()-size.width())/2,(height()-size.height())/2),size);
  }
  void paintEvent(QPaintEvent *) override {
    QPainter p(this); p.fillRect(rect(),Qt::black); auto r=image_rect();
    p.drawImage(r,image_); p.setRenderHint(QPainter::Antialiasing);
    QPolygonF poly;
    for(size_t i=0;i<points.size();++i) {
      QPointF q(r.x()+points[i].x*r.width()/image_.width(),r.y()+points[i].y*r.height()/image_.height());
      poly<<q; p.setPen(QPen(Qt::yellow,2));p.setBrush(Qt::yellow);p.drawEllipse(q,5,5);
      p.drawText(q+QPointF(7,-7),QString::number(i+1));
    }
    p.setPen(QPen(Qt::cyan,2)); p.setBrush(Qt::NoBrush); p.drawPolygon(poly);
  }
  void mousePressEvent(QMouseEvent *event) override {
    if(event->button()==Qt::RightButton){points.clear();update();return;}
    const auto r=image_rect(); if(!r.contains(event->pos()))return;
    if(points.size()==4)points.clear();
    points.emplace_back(float(event->x()-r.x())*image_.width()/r.width(),float(event->y()-r.y())*image_.height()/r.height());
    update();
  }
private: QImage image_;
};
}

AdasController::AdasController(MainWindow *window,pango_fun *device,Ui::MainWindow *ui)
    :QObject(window),window_(window),device_(device),ui_(ui) {
  started_s_=last_stats_s_=adas::monotonic_seconds();
  const QString manifest_override=qEnvironmentVariable("ADAS_MODEL_MANIFEST");
  const QString manifest=manifest_override.isEmpty()?QDir(project_root()).filePath("models/adas/manifest.json"):manifest_override;
  adas::NeuralBundleSpec bundle;QString manifest_error;
  if(adas::read_neural_manifest(manifest,bundle,manifest_error)&&bundle.dual) {
    engine_.required_obstacle_components=adas::ObjectObservation::Combined;
    engine_.separate_crosswalk_source=true;
  }
  detector_=new adas::DetectionWorker(this);
  geometry_=new adas::GeometryWorker(this);
  geometry_->set_enabled(true);
  connect(geometry_,&adas::GeometryWorker::result_ready,this,&AdasController::queue_geometry,Qt::DirectConnection);
  detector_->set_model_root(project_root());
  connect(detector_,&adas::DetectionWorker::result_ready,this,&AdasController::queue_result,Qt::DirectConnection);
  // Execute this callback in the capture emitter thread before its next read().
  connect(device_,&pango_fun::send_wr_done,this,&AdasController::ingest_live,Qt::DirectConnection);
  setup_controls(); load_profile(); update_settings();
  detector_->set_mode(DetectionMode::Adas); detector_->start();geometry_->start();
  qApp->installEventFilter(this);
  timer_.setInterval(33); connect(&timer_,&QTimer::timeout,this,&AdasController::tick);timer_.start();
}
AdasController::~AdasController() { shutdown(); }

void AdasController::setup_controls() {
  // Existing FPGA controls remain available inside scrolling side panels.
  for(auto *panel:{ui_->controlPanel,ui_->infoPanel}) {
    panel->setMinimumWidth(205); panel->setMaximumWidth(245);
    auto *scroll=new QScrollArea(window_);scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);scroll->setFrameShape(QFrame::NoFrame);
    scroll->setMinimumWidth(215);scroll->setMaximumWidth(255);
    ui_->centralLayout->replaceWidget(panel,scroll);scroll->setWidget(panel);
  }
  ui_->Label->setMinimumSize(320,180);
  window_->setMinimumSize(900,540);window_->resize(1280,720);
  window_->setWindowTitle(QStringLiteral("Pango FPGA + RK3568 · ADAS 辅助驾驶"));
  QVBoxLayout *l=nullptr;
  auto *source=group(QStringLiteral("ADAS 视频与视图"),window_,&l);
  mosaic_check_=new QCheckBox(QStringLiteral("四路画布（前/右/后/左）"),source);mosaic_check_->setChecked(true);l->addWidget(mosaic_check_);
  mosaic_check_->setEnabled(false);
  mosaic_check_->setToolTip(QStringLiteral("布局跟随 FPGA 实际输入源；放大前视不改变四路安全检查"));
  front_check_=new QCheckBox(QStringLiteral("放大前视 [V]"),source);l->addWidget(front_check_);
  connect(front_check_,&QCheckBox::toggled,this,[this](bool checked){front_only_=checked;});
  source_label_=wrapping_label(source);source_label_->setObjectName("adasSourceStatus");source_label_->setText(QStringLiteral("等待 FPGA 采集\n电脑播放素材 → HDMI → FPGA → PCIe → ARM"));l->addWidget(source_label_);
  ui_->controlPanelLayout->insertWidget(0,source);
  auto *controls=group(QStringLiteral("模拟驾驶控制"),window_,&l);
  auto *form=new QFormLayout;form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);l->addLayout(form);
  lane_combo_=new QComboBox(controls);lane_combo_->addItems({QStringLiteral("左车道 [1]"),QStringLiteral("直行车道 [2]"),QStringLiteral("右车道 [3]")});lane_combo_->setCurrentIndex(1);form->addRow(QStringLiteral("本车"),lane_combo_);
  signal_combo_=new QComboBox(controls);signal_combo_->addItems({QStringLiteral("关闭"),QStringLiteral("左 [A]"),QStringLiteral("右 [D]")});form->addRow(QStringLiteral("转向灯"),signal_combo_);
  force_lane_combo_=new QComboBox(controls);force_lane_combo_->addItems({QStringLiteral("无"),QStringLiteral("强制左 [J]"),QStringLiteral("强制右 [L]")});form->addRow(QStringLiteral("变道"),force_lane_combo_);
  nav_combo_=new QComboBox(controls);nav_combo_->addItems({QStringLiteral("不跟随导航"),QStringLiteral("手动左"),QStringLiteral("手动直行"),QStringLiteral("手动右")});form->addRow(QStringLiteral("导航 [N]"),nav_combo_);
  auto_check_=new QCheckBox(QStringLiteral("自动模拟驾驶"),controls);auto_check_->setChecked(true);l->addWidget(auto_check_);
  intersection_check_=new QCheckBox(QStringLiteral("路口方向提示"),controls);l->addWidget(intersection_check_);
  manual_stop_check_=new QCheckBox(QStringLiteral("人工停车 [↓ / 空格]"),controls);l->addWidget(manual_stop_check_);
  force_drive_check_=new QCheckBox(QStringLiteral("强制行驶 [Ctrl+↑]"),controls);l->addWidget(force_drive_check_);
  speed_spin_=new QDoubleSpinBox(controls);speed_spin_->setRange(0,180);speed_spin_->setValue(0);speed_spin_->setSuffix(" km/h");form->addRow(QStringLiteral("仿真速度"),speed_spin_);
  limit_spin_=new QDoubleSpinBox(controls);limit_spin_->setRange(1,180);limit_spin_->setValue(60);limit_spin_->setSuffix(" km/h");form->addRow(QStringLiteral("限速"),limit_spin_);
  for(auto *combo:{lane_combo_,signal_combo_,force_lane_combo_})connect(combo,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int){update_ego();});
  const auto select_navigation=[this](int index){
    // A navigation selection is the same ordinary operator action as an arrow
    // key. It must not inherit a previous forced manoeuvre or opposite signal.
    using Action=adas::ManualAction;
    const Action actions[]={Action::ClearNavigation,Action::NavigateLeft,Action::NavigateStraight,Action::NavigateRight};
    if(index>=0&&index<4)apply_manual(actions[index]);
  };
  connect(nav_combo_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,select_navigation);
  connect(nav_combo_,QOverload<int>::of(&QComboBox::activated),this,[this,select_navigation](int index){
    if(index<0||index>3)return;
    const auto signal=index==1?adas::TurnSignal::Left:index==3?adas::TurnSignal::Right:adas::TurnSignal::None;
    // changed already handles a new index; activated also covers selecting the
    // same item after a forced action, without applying a changed item twice.
    if(ego_.force_drive||ego_.force_lane_change!=adas::TurnSignal::None||ego_.signal!=signal||
       ego_.manual_nav_enabled!=(index>0)||(index>0&&int(ego_.manual_nav)!=index))select_navigation(index);
    else refresh_turn_request(true);
  });
  for(auto *check:{auto_check_,intersection_check_,manual_stop_check_,force_drive_check_})connect(check,&QCheckBox::toggled,this,[this](bool){update_ego();});
  for(auto *spin:{speed_spin_,limit_spin_})connect(spin,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this](double){update_ego();});
  auto *reset=new QPushButton(QStringLiteral("复位场景与决策"),controls);reset->setObjectName("adasResetDecision");l->addWidget(reset);connect(reset,&QPushButton::clicked,this,&AdasController::reset_decision);
  ui_->infoPanelLayout->insertWidget(0,controls);
  auto *calib=group(QStringLiteral("标定与感知区域"),window_,&l);
  auto *calibrate_button=new QPushButton(QStringLiteral("前视路面四点标定"),calib);calibrate_button->setObjectName("adasCalibrate");l->addWidget(calibrate_button);connect(calibrate_button,&QPushButton::clicked,this,&AdasController::calibrate);
  auto *roi=new QPushButton(QStringLiteral("各车道信号灯 ROI"),calib);l->addWidget(roi);connect(roi,&QPushButton::clicked,this,&AdasController::edit_rois);
  auto *clear=new QPushButton(QStringLiteral("清除当前标定"),calib);l->addWidget(clear);connect(clear,&QPushButton::clicked,this,[this](){calibration_.reset();QFile::remove(project_root()+"/config/calibrations/"+profile_key()+".yml");update_settings();advance_generation();});
  calibration_label_=wrapping_label(calib);l->addWidget(calibration_label_);
  auto *export_report=new QPushButton(QStringLiteral("导出诊断报告…"),calib);export_report->setObjectName("adasExportReport");l->addWidget(export_report);
  connect(export_report,&QPushButton::clicked,this,[this](){
    const auto path=QFileDialog::getSaveFileName(window_,QStringLiteral("导出 ADAS 诊断报告"),QStringLiteral("adas-report.json"),QStringLiteral("JSON (*.json)"));
    if(!path.isEmpty())write_report(path);
  });
  ui_->controlPanelLayout->insertWidget(1,calib);
  update_ego();
  lane_combo_->setObjectName("adasLane");signal_combo_->setObjectName("adasTurnSignal");force_lane_combo_->setObjectName("adasForceLane");
  nav_combo_->setObjectName("adasNavigation");manual_stop_check_->setObjectName("adasManualStop");force_drive_check_->setObjectName("adasForceDrive");
  speed_spin_->setObjectName("adasManualSpeed");limit_spin_->setObjectName("adasSpeedLimit");
  mosaic_check_->setObjectName("adasMosaic");front_check_->setObjectName("adasFrontView");
}

void AdasController::ingest_live(int slot) {
  const auto generation=generation_.load();
  if(!accepting_live_.load()||slot<0||slot>3)return;
  const double now=adas::monotonic_seconds();
  if(now<discard_live_before_.load())return;
  adas::Stamp stamp{generation,++sequence_,now,now,false};
  auto frame=live_pool_.copy(device_->img_buf+static_cast<size_t>(slot)*1280*720*2,
    1280,720,2560,stamp,static_cast<adas::Layout>(layout_.load()));
  if(frame)publish(frame);else ++pool_dropped_;
}
void AdasController::publish(const adas::FramePacket &input) {
  if(!input)return;
  adas::FramePacket frame=input;
  if(frame.stamp.generation!=generation_.load())return;
  frame.stamp.sequence=++sequence_;
  ++captured_;
  display_mailbox_.publish(frame);detector_->submit(frame);
  geometry_->submit(frame); // Worker mutex owns its enabled/mode gate; capture thread reads no widgets.
}
void AdasController::advance_generation() {
  const auto gen=++generation_;engine_.reset(gen);
  display_mailbox_.take();
  {
    std::lock_guard<std::mutex> lock(results_mutex_);
    pending_results_={};pending_result_ready_={};pending_geometry_={};pending_geometry_ready_=false;
  }
  views_={};vehicle_views_={};roadblock_views_={};crosswalk_views_={};last_geometry_={};last_signal_={};decision_={};
  displayed_={};rendered_=QPixmap();ui_->Label->clear();
  geometry_->set_enabled(false);geometry_->set_enabled(mode_==DetectionMode::Adas);
  requested_turn_=adas::TurnSignal::None;ego_.lane_change_request_s=0;
  refresh_turn_request();
}
bool AdasController::stop_live() {
  accepting_live_=false;
  if(!device_->isRunning()){ui_->connect->setText(QStringLiteral("采集视频"));return true;}
  device_->stopped=true;
  if(device_->dev_fd>=0&&device_->mem_pio_addr)
    device_->pio_write(0xffffff00,0,0); // Original verified stop command.
  if(!device_->wait(2000)) {
    window_->statusBar()->showMessage(QStringLiteral("采集线程尚未退出；保留映射与缓冲，拒绝切源或关闭"));
    return false;
  }
  ui_->connect->setText(QStringLiteral("采集视频"));
  return true;
}
bool AdasController::stop_source() {
  if(!stop_live())return false;
  advance_generation();return true;
}
bool AdasController::shutdown() {
  if(closing_)return true;
  if(!stop_live())return false;
  closing_=true;timer_.stop();
  disconnect(device_,nullptr,this,nullptr);
  detector_->stop();detector_->wait();geometry_->stop();geometry_->wait();
  qApp->removeEventFilter(this);return true;
}
bool AdasController::prepare_live() {
  if(device_->isRunning()&&!accepting_live_) {
    if(!stop_live())return false;
  }
  // Reopening capture cannot reuse another capture session's source identity.
  accepting_live_=false;live_source_known_=false;
  update_ego();
  sync_live_source();load_profile();update_settings();advance_generation();accepting_live_=true;
  source_label_->setText(QStringLiteral("FPGA → PCIe · 1280×720 RGB565\n时间基准：ARM 接收单调时间"));return true;
}
void AdasController::set_layout(adas::Layout layout) {
  if(live_source_known_)layout=ui_->radioHDMI->isChecked()?adas::Layout::Single:adas::Layout::Mosaic;
  const int value=static_cast<int>(layout);
  if(layout_.load()==value)return;
  const bool was_live=accepting_live_.exchange(false);
  layout_=value;{QSignalBlocker b(mosaic_check_);mosaic_check_->setChecked(layout==adas::Layout::Mosaic);}
  update_ego();load_profile();update_settings();advance_generation();
  accepting_live_=was_live;
}
void AdasController::set_mode(DetectionMode mode) {
  if(mode!=DetectionMode::Adas&&mode!=DetectionMode::RawVideo)return;
  if(mode_==mode)return;
  mode_=mode;detector_->set_mode(mode);
  advance_generation();
}
void AdasController::physical_source_changed() {
  const bool was_live=accepting_live_.exchange(false);
  // A successful select write is not yet evidence that a complete frame from
  // that source has reached DMA. Confirm through the existing status read only
  // after the established settling interval; preserve the original buttons.
  live_source_known_=false;
  discard_live_before_=adas::monotonic_seconds()+.25;
  load_profile();update_settings();advance_generation();
  source_label_->setText(QStringLiteral("FPGA 视频源切换中：等待状态确认与新帧"));
  accepting_live_=was_live;
}
void AdasController::sync_live_source() {
  if(device_->dev_fd<0) {
    if(live_source_known_) {
      live_source_known_=false;
      load_profile();update_settings();advance_generation();
      source_label_->setText(QStringLiteral("FPGA 设备未打开：等待重新确认输入源"));
    }
    return;
  }
  if(adas::monotonic_seconds()<discard_live_before_.load())return;
  // Reuse the verified status-read API; no source-select register write.
  const int source=device_->i2c_query_source();
  const bool known=source==0||source==1;
  const auto actual_layout=source==1?adas::Layout::Single:adas::Layout::Mosaic;
  const bool changed=known!=live_source_known_ || (known&&(ui_->radioHDMI->isChecked()!=(source==1)||layout_.load()!=int(actual_layout)));
  if(!changed)return;
  live_source_known_=known;
  if(known) {
    QSignalBlocker camera(ui_->radioCamera),hdmi(ui_->radioHDMI),mosaic(mosaic_check_);
    ui_->radioCamera->setChecked(source==0);ui_->radioHDMI->setChecked(source==1);
    layout_=int(actual_layout);mosaic_check_->setChecked(actual_layout==adas::Layout::Mosaic);update_ego();
  }
  const bool was_live=accepting_live_.exchange(false);
  load_profile();update_settings();advance_generation();
  discard_live_before_=adas::monotonic_seconds()+.25;accepting_live_=was_live;
  source_label_->setText(known?(source==1?QStringLiteral("FPGA HDMI · 单路前视"):QStringLiteral("FPGA 摄像头 · 四路画布")):QStringLiteral("FPGA 视频源状态未知，来源标定已停用"));
  window_->statusBar()->showMessage(known?QStringLiteral("已同步 FPGA 实际视频源；旧标定/感知历史失效"):
    QStringLiteral("无法确认 FPGA 视频源；停用来源标定，等待 I2C 状态恢复"),5000);
}
QString AdasController::profile_key() const {
  QString identity=!live_source_known_?QStringLiteral("pcie-unknown"):
    ui_->radioHDMI->isChecked()?QStringLiteral("pcie-hdmi"):QStringLiteral("pcie-camera");
  identity+=layout_.load()==0?"|mosaic":"|single";
  return QString::fromLatin1(QCryptographicHash::hash(identity.toUtf8(),QCryptographicHash::Sha256).toHex().left(24));
}
void AdasController::load_profile() {
  const int w=layout_.load()==0?640:1280,h=layout_.load()==0?360:720;
  const auto key=profile_key();std::string error;
  calibration_.load((project_root()+"/config/calibrations/"+key+".yml").toStdString(),w,h,&error);
  if(!live_source_known_)calibration_.reset();
  if(!calibration_.is_compatible(w,h,key.toStdString()))calibration_.reset();
  QSettings settings(config_path(),QSettings::IniFormat);settings.setIniCodec("UTF-8");
  load_timing(settings);
  settings.beginGroup("profiles/"+key);
  auto read_roi=[&settings](const QString &name,adas::Roi fallback){
    auto values=settings.value(name).toList();if(values.size()!=4)return fallback;
    adas::Roi roi{values[0].toDouble(),values[1].toDouble(),values[2].toDouble(),values[3].toDouble()};
    if(!std::isfinite(roi.x)||!std::isfinite(roi.y)||!std::isfinite(roi.w)||!std::isfinite(roi.h)||roi.x<0||roi.y<0||roi.w<=0||roi.h<=0||roi.x+roi.w>1||roi.y+roi.h>1)return fallback;
    return roi;
  };
  for(int i=0;i<3;++i)light_rois_[i]=read_roi("signal_"+QString::number(i),{0,0,1,.5});
}
void AdasController::update_settings() {
  detector_->configure(calibration_,light_rois_);
  geometry_->configure(calibration_);
  if(calibration_label_)calibration_label_->setText(calibration_.valid()?QStringLiteral("标定有效：%1×%2 m\n距离：%3")
    .arg(calibration_.width_m()).arg(calibration_.length_m()).arg(calibration_.distance_valid()?QStringLiteral("有效"):QStringLiteral("未设置车头偏移")):QStringLiteral("未标定：米制距离不可用；速度使用独立来源"));
}
void AdasController::update_ego() {
  const auto old_signal=ego_.signal,old_force=ego_.force_lane_change;
  const bool old_manual=ego_.manual_nav_enabled;const auto old_nav=ego_.manual_nav;
  ego_.input_layout=static_cast<adas::Layout>(layout_.load());
  ego_.lane=static_cast<adas::LaneId>(lane_combo_->currentIndex());ego_.signal=static_cast<adas::TurnSignal>(signal_combo_->currentIndex());
  ego_.force_lane_change=static_cast<adas::TurnSignal>(force_lane_combo_->currentIndex());
  ego_.manual_nav_enabled=nav_combo_->currentIndex()>0;
  ego_.manual_nav=static_cast<adas::Direction>(std::max(1,nav_combo_->currentIndex()));
  ego_.automatic=auto_check_->isChecked();ego_.intersection_mode=intersection_check_->isChecked();
  ego_.manual_stop=manual_stop_check_->isChecked();ego_.force_drive=force_drive_check_->isChecked();
  ego_.use_visual_speed=false;ego_.speed_source=adas::SpeedSource::Simulation;
  ego_.manual_speed_kmh=float(speed_spin_->value());ego_.speed_limit=float(limit_spin_->value());
  ego_.paused=false;
  refresh_turn_request(old_signal!=ego_.signal||old_force!=ego_.force_lane_change||old_manual!=ego_.manual_nav_enabled||old_nav!=ego_.manual_nav);
}
void AdasController::apply_manual(adas::ManualAction action) {
  auto next=ego_;adas::apply_manual_action(next,action);
  {
    const QSignalBlocker signal(signal_combo_),force(force_lane_combo_),nav(nav_combo_),stop(manual_stop_check_),drive(force_drive_check_),automatic(auto_check_);
    signal_combo_->setCurrentIndex(int(next.signal));force_lane_combo_->setCurrentIndex(int(next.force_lane_change));
    nav_combo_->setCurrentIndex(next.manual_nav_enabled?int(next.manual_nav):0);
    manual_stop_check_->setChecked(next.manual_stop);force_drive_check_->setChecked(next.force_drive);auto_check_->setChecked(next.automatic);
  }
  update_ego();refresh_turn_request(true); // A repeated deliberate request needs a new target-side observation.
}
void AdasController::refresh_turn_request(bool new_request) {
  const double now=adas::monotonic_seconds();
  adas::TurnSignal intent=ego_.force_lane_change;
  const bool junction=ego_.intersection_mode||decision_.light!=adas::LightState::Unknown;
  if(intent==adas::TurnSignal::None&&!junction) {
    intent=ego_.signal;
    if(intent==adas::TurnSignal::None) {
      const auto nav=ego_.manual_nav_enabled?ego_.manual_nav:adas::Direction::None;
      intent=nav==adas::Direction::Left?adas::TurnSignal::Left:nav==adas::Direction::Right?adas::TurnSignal::Right:adas::TurnSignal::None;
    }
  }
  if(intent==adas::TurnSignal::None)ego_.lane_change_request_s=0;
  else if(intent!=requested_turn_||new_request)ego_.lane_change_request_s=now;
  if(intent!=requested_turn_)detector_->set_intent(intent);
  requested_turn_=intent;
}
void AdasController::reset_decision() {
  signal_combo_->setCurrentIndex(0);force_lane_combo_->setCurrentIndex(0);nav_combo_->setCurrentIndex(0);
  manual_stop_check_->setChecked(false);force_drive_check_->setChecked(false);
  advance_generation();
}
// Direct worker callbacks only replace bounded latest metadata. All decisions,
// counters, and widget access stay on the GUI thread in drain_results().
void AdasController::queue_result(VisionFrameResult result) {
  std::lock_guard<std::mutex> lock(results_mutex_);
  if(result.stamp.generation!=generation_.load())return;
  if(result.mode!=DetectionMode::Adas)return;
  const int view=int(result.view);if(view<0||view>3)return;
  for(int component=0;component<5;++component) {
    const unsigned bit=1u<<component;
    if(!(result.observation.components&bit)||(component==3&&view!=0))continue;
    auto &slot=pending_results_[view][component];auto &ready=pending_result_ready_[view][component];
    if(ready&&result.stamp.sequence<=slot.stamp.sequence)continue;
    if(ready)++result_overwritten_;
    slot=result;slot.observation.components=bit;ready=true;
  }
}
void AdasController::queue_geometry(adas::GeometryObservation result) {
  std::lock_guard<std::mutex> lock(results_mutex_);
  if(result.stamp.generation!=generation_.load()||!(result.components&adas::GeometryObservation::Lane))return;
  if(pending_geometry_ready_&&result.stamp.sequence<=pending_geometry_.stamp.sequence)return;
  if(pending_geometry_ready_)++result_overwritten_;
  pending_geometry_=std::move(result);pending_geometry_.components=adas::GeometryObservation::Lane;
  pending_geometry_ready_=true;
}
void AdasController::drain_results() {
  std::array<std::array<VisionFrameResult,5>,4> results{};
  std::array<std::array<bool,5>,4> ready{};
  adas::GeometryObservation geometry;bool geometry_ready=false;
  {
    std::lock_guard<std::mutex> lock(results_mutex_);
    results.swap(pending_results_);ready.swap(pending_result_ready_);
    std::swap(geometry,pending_geometry_);std::swap(geometry_ready,pending_geometry_ready_);
  }
  for(int view=0;view<4;++view)for(int component=0;component<5;++component)
    if(ready[view][component])accept_result(std::move(results[view][component]));
  if(geometry_ready)accept_geometry(std::move(geometry));
}
void AdasController::accept_result(VisionFrameResult result) {
  if(closing_||mode_!=DetectionMode::Adas||result.mode!=DetectionMode::Adas||result.stamp.generation!=generation_.load())return;
  const int v=static_cast<int>(result.view);if(v<0||v>3)return;
  if(result.observation.stamp.generation!=result.stamp.generation||result.observation.stamp.sequence!=result.stamp.sequence)return;
  if(engine_.generation()!=generation_.load())engine_.reset(generation_.load());
  auto component_result=[&](unsigned component) {
    auto split=result;split.observation.components=component;
    split.overlays.erase(std::remove_if(split.overlays.begin(),split.overlays.end(),[&](const OverlayBoxResult &box) {
      return component==adas::ObjectObservation::Vehicle?box.class_id!=10:
        component==adas::ObjectObservation::Roadblock?box.class_id!=11:
        component==adas::ObjectObservation::Crosswalk?box.class_id==1||box.class_id==10||box.class_id==11:
        engine_.separate_crosswalk_source?box.class_id!=1:box.class_id==10||box.class_id==11;
    }),split.overlays.end());
    return split;
  };
  unsigned accepted=0;
  if((result.observation.components&adas::ObjectObservation::Traffic)&&result.stamp.sequence>views_[v].stamp.sequence) {
    views_[v]=component_result(adas::ObjectObservation::Traffic);if(result.observation.traffic_status==adas::Status::Valid)++detection_count_[v];accepted|=adas::ObjectObservation::Traffic;
  }
  if((result.observation.components&adas::ObjectObservation::Vehicle)&&result.stamp.sequence>vehicle_views_[v].stamp.sequence) {
    vehicle_views_[v]=component_result(adas::ObjectObservation::Vehicle);if(result.observation.vehicle_status==adas::Status::Valid)++vehicle_count_[v];accepted|=adas::ObjectObservation::Vehicle;
  }
  if((result.observation.components&adas::ObjectObservation::Roadblock)&&result.stamp.sequence>roadblock_views_[v].stamp.sequence) {
    roadblock_views_[v]=component_result(adas::ObjectObservation::Roadblock);accepted|=adas::ObjectObservation::Roadblock;
  }
  if((result.observation.components&adas::ObjectObservation::Crosswalk)&&result.stamp.sequence>crosswalk_views_[v].stamp.sequence) {
    crosswalk_views_[v]=component_result(adas::ObjectObservation::Crosswalk);
    if(result.observation.crosswalk_status==adas::Status::Valid)++crosswalk_count_[v];
    accepted|=adas::ObjectObservation::Crosswalk;
  }
  const bool signal=v==0&&((result.observation.components&adas::ObjectObservation::Signal)||
    ((result.observation.components&adas::ObjectObservation::Traffic)&&result.observation.signal_observation&&!engine_.separate_crosswalk_source));
  if(signal&&result.stamp.sequence>last_signal_.stamp.sequence) {
    const auto &p=result.observation;
    const auto status=p.signal_status==adas::Status::Missing&&(p.components&adas::ObjectObservation::Traffic)?p.traffic_status:p.signal_status;
    last_signal_={result.stamp,status,p.light,p.has_lane_lights,p.lane_lights};if(status==adas::Status::Valid)++signal_count_;
  }
  if(result.observation.components&adas::ObjectObservation::Signal)accepted|=adas::ObjectObservation::Signal;
  if(accepted){result.observation.components=accepted;engine_.ingest(result.observation,adas::monotonic_seconds());}
}
void AdasController::accept_geometry(adas::GeometryObservation result) {
  if(closing_||mode_!=DetectionMode::Adas||result.stamp.generation!=generation_.load())return;
  if(engine_.generation()!=generation_.load())engine_.reset(generation_.load());
  unsigned accepted=0;
  if((result.components&adas::GeometryObservation::Lane)&&result.stamp.sequence>last_geometry_.stamp.sequence) {
    last_geometry_=result;last_geometry_.components=adas::GeometryObservation::Lane;if(result.status==adas::Status::Valid)++geometry_count_;accepted|=adas::GeometryObservation::Lane;
  }
  // Navigation is manual: visual results cannot change its direction or confirmation history.
  if(accepted){result.components=accepted;engine_.ingest(result,adas::monotonic_seconds());refresh_turn_request();}
}
bool AdasController::eventFilter(QObject *receiver,QEvent *event) {
  if(event->type()!=QEvent::KeyPress||mode_!=DetectionMode::Adas)return false;
  auto editing=[](QObject *object) {
    for(auto *widget=qobject_cast<QWidget*>(object);widget;widget=widget->parentWidget())
      if(qobject_cast<QLineEdit*>(widget)||qobject_cast<QAbstractSpinBox*>(widget)||qobject_cast<QComboBox*>(widget)||qobject_cast<QTextEdit*>(widget))return true;
    return false;
  };
  if(editing(receiver)||editing(qApp->focusWidget()))return false;
  auto *key=static_cast<QKeyEvent*>(event);if(key->isAutoRepeat())return false;
  const bool control=key->modifiers().testFlag(Qt::ControlModifier);
  using Action=adas::ManualAction;
  switch(key->key()) {
  case Qt::Key_1:case Qt::Key_2:case Qt::Key_3:lane_combo_->setCurrentIndex(key->key()-Qt::Key_1);break;
  case Qt::Key_Up:apply_manual(control?Action::ForceDrive:Action::Drive);break;
  case Qt::Key_Down:apply_manual(Action::Stop);break;
  case Qt::Key_Left:apply_manual(control?Action::ForceChangeLeft:Action::NavigateLeft);break;
  case Qt::Key_Right:apply_manual(control?Action::ForceChangeRight:Action::NavigateRight);break;
  case Qt::Key_W:apply_manual(Action::NavigateStraight);break;
  case Qt::Key_Escape:apply_manual(Action::ClearNavigation);break;
  case Qt::Key_A:apply_manual(Action::SignalLeft);break;
  case Qt::Key_D:apply_manual(Action::SignalRight);break;
  case Qt::Key_J:apply_manual(Action::ToggleForceLeft);break;
  case Qt::Key_L:apply_manual(Action::ToggleForceRight);break;
  case Qt::Key_Space:apply_manual(Action::ToggleStop);break;
  case Qt::Key_N:apply_manual(Action::CycleNavigation);break;
  case Qt::Key_V:front_check_->toggle();break;
  default:return false;
  }
  return true;
}

void AdasController::calibrate() {
  if(!displayed_){QMessageBox::information(window_,QStringLiteral("标定"),QStringLiteral("请先通过 FPGA 采集一帧道路图像"));return;}
  if(!live_source_known_){QMessageBox::information(window_,QStringLiteral("标定"),QStringLiteral("无法确认 FPGA 实际视频源，请先恢复 I2C 状态或从界面选择视频源"));return;}
  const auto calibration_profile=profile_key();
  const int calibration_layout=layout_.load();
  const auto r=adas::view_rect(displayed_.layout,adas::ViewId::Front,displayed_.width,displayed_.height);
  const auto *data=displayed_.pixels->bytes.data()+r.y*displayed_.stride+r.x*2;
  QImage image(data,r.w,r.h,displayed_.stride,QImage::Format_RGB16);
  QDialog dialog(window_);dialog.setWindowTitle(QStringLiteral("前视路面标定 · 使用实测尺寸"));dialog.resize(850,560);
  auto *layout=new QVBoxLayout(&dialog);
  auto *help=new QLabel(QStringLiteral("依次点击：左上→右上→右下→左下。右键清除。\n选择同一平面上的已知矩形；宽、长须填写实测米数。未测车头偏移时不输出米制目标距离。"),&dialog);help->setWordWrap(true);layout->addWidget(help);
  auto *canvas=new CalibrationCanvas(image.copy(),&dialog);layout->addWidget(canvas,1);
  auto *form=new QFormLayout;layout->addLayout(form);
  auto spin=[&](double low,double high,double value){auto *s=new QDoubleSpinBox(&dialog);s->setDecimals(3);s->setRange(low,high);s->setValue(value);s->setSuffix(" m");return s;};
  auto *width=spin(.1,1000,calibration_.valid()?calibration_.width_m():3.5);
  auto *length=spin(.1,10000,calibration_.valid()?calibration_.length_m():6);
  auto *offset=spin(-1,1000,calibration_.valid()?calibration_.near_offset_m():-1);offset->setSpecialValueText(QStringLiteral("未测量"));
  form->addRow(QStringLiteral("矩形实测宽"),width);form->addRow(QStringLiteral("矩形实测长"),length);
  form->addRow(QStringLiteral("车头到近边距离"),offset);
  auto *buttons=new QDialogButtonBox(QDialogButtonBox::Save|QDialogButtonBox::Cancel,&dialog);layout->addWidget(buttons);
  connect(buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);
  connect(buttons,&QDialogButtonBox::accepted,&dialog,[&](){
    if(profile_key()!=calibration_profile||layout_.load()!=calibration_layout||!live_source_known_) {
      QMessageBox::warning(&dialog,QStringLiteral("标定来源变化"),QStringLiteral("标定来源或布局已变化，请关闭后重新打开标定"));return;
    }
    if(canvas->points.size()!=4){QMessageBox::warning(&dialog,QStringLiteral("标定"),QStringLiteral("请按顺序选择四个点"));return;}
    std::array<cv::Point2f,4> points;std::copy(canvas->points.begin(),canvas->points.end(),points.begin());
    adas::GroundCalibration candidate;std::string error;
    if(!candidate.configure(points,width->value(),length->value(),offset->value(),r.w,r.h,&error)) {
      QMessageBox::warning(&dialog,QStringLiteral("标定无效"),QString::fromStdString(error));return;
    }
    candidate.set_profile_identity(calibration_profile.toStdString());
    QDir().mkpath(project_root()+"/config/calibrations");
    const bool saved=candidate.save((project_root()+"/config/calibrations/"+calibration_profile+".yml").toStdString());
    calibration_=candidate;update_settings();advance_generation();dialog.accept();
    if(!saved)window_->statusBar()->showMessage(QStringLiteral("标定已应用，保存失败；请检查配置目录"));
  });
  dialog.exec();
}
void AdasController::edit_rois() {
  QDialog dialog(window_);dialog.setWindowTitle(QStringLiteral("前视 ROI · 相对前视图的百分比"));
  auto *layout=new QVBoxLayout(&dialog);auto *grid=new QGridLayout;layout->addLayout(grid);
  const QStringList names={QStringLiteral("左车道灯"),QStringLiteral("直行灯"),QStringLiteral("右车道灯")};
  std::array<std::array<QDoubleSpinBox*,4>,3> values{};
  for(int col=0;col<4;++col)grid->addWidget(new QLabel(QStringList{"X%","Y%","W%","H%"}[col],&dialog),0,col+1);
  for(int row=0;row<3;++row) {
    grid->addWidget(new QLabel(names[row],&dialog),row+1,0);
    auto roi=light_rois_[row];double initial[]={roi.x,roi.y,roi.w,roi.h};
    for(int col=0;col<4;++col) {
      auto *spin=new QDoubleSpinBox(&dialog);spin->setRange(col<2?0:1,100);spin->setValue(initial[col]*100);
      spin->setDecimals(1);values[row][col]=spin;grid->addWidget(spin,row+1,col+1);
    }
  }
  auto *help=new QLabel(QStringLiteral("X/Y 为左上角，W/H 为宽高。各车道信号区域应避开其他方向灯；默认使用前视上半幅。"),&dialog);help->setWordWrap(true);layout->addWidget(help);
  auto *buttons=new QDialogButtonBox(QDialogButtonBox::Save|QDialogButtonBox::Cancel,&dialog);layout->addWidget(buttons);
  connect(buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);
  connect(buttons,&QDialogButtonBox::accepted,&dialog,[&](){
    std::array<adas::Roi,3> rois;
    for(int row=0;row<3;++row) {
      rois[row]={values[row][0]->value()/100,values[row][1]->value()/100,values[row][2]->value()/100,values[row][3]->value()/100};
      if(rois[row].x+rois[row].w>1.00001||rois[row].y+rois[row].h>1.00001){QMessageBox::warning(&dialog,QStringLiteral("ROI"),QStringLiteral("区域不能超出前视边界"));return;}
    }
    light_rois_=rois;
    QDir().mkpath(project_root()+"/config");QSettings settings(config_path(),QSettings::IniFormat);settings.setIniCodec("UTF-8");settings.beginGroup("profiles/"+profile_key());
    for(int i=0;i<3;++i)settings.setValue("signal_"+QString::number(i),QVariantList{rois[i].x,rois[i].y,rois[i].w,rois[i].h});
    settings.sync();update_settings();advance_generation();dialog.accept();
  });dialog.exec();
}

void AdasController::tick() {
  if(closing_)return;
  auto frame=display_mailbox_.take();
  if(frame&&frame.stamp.generation==generation_.load())displayed_=std::move(frame);
  drain_results();
  const double wall=adas::monotonic_seconds(),now=wall;
  if(engine_.generation()!=generation_.load())engine_.reset(generation_.load());
  refresh_turn_request();
  auto evaluated_ego=ego_;
  const bool unconfirmed_source=accepting_live_.load()&&!live_source_known_;
  if(unconfirmed_source)evaluated_ego.manual_stop=true;
  decision_=engine_.evaluate(evaluated_ego,now);
  if(unconfirmed_source) {
    decision_.status_text=u8"FPGA 输入源未确认：停车等待";
    decision_.hint_text=u8"等待原有 I2C 状态确认输入布局";
  }
  if(wall-last_stats_s_>=1)refresh_stats(wall);
  render(now);
}
void AdasController::render(double now) {
  if(!displayed_)return;
  const bool adas_mode=mode_==DetectionMode::Adas;
  const bool crop=adas_mode&&displayed_.layout==adas::Layout::Mosaic&&front_only_;
  QImage image(displayed_.pixels->bytes.data(),displayed_.width,displayed_.height,displayed_.stride,QImage::Format_RGB16);
  if(crop)image=image.copy(0,0,displayed_.width/2,displayed_.height/2);
  QPixmap output=QPixmap::fromImage(image);QPainter painter(&output);painter.setRenderHint(QPainter::Antialiasing);
  const auto display_size=output.size().scaled(ui_->Label->size(),Qt::KeepAspectRatio);
  const double text_scale=std::max(1.,double(output.width())/std::max(1,display_size.width()));
  const uint64_t gen=generation_.load();
  const auto fresh=[&](const adas::Stamp &s,double timeout){return s.generation==gen&&s.sequence&&now>=s.receive_s&&(s.static_image||now-s.receive_s<=timeout);};
  const int views=adas_mode&&displayed_.layout==adas::Layout::Mosaic&&!crop?4:1;
  const QStringList names={QStringLiteral("前视"),QStringLiteral("右视"),QStringLiteral("后视"),QStringLiteral("左视")};
  auto draw_boxes=[&](const VisionFrameResult &result) {
    QFont font=painter.font();font.setPixelSize(qRound(12*text_scale));font.setBold(true);painter.setFont(font);
    for(const auto &box:result.overlays) {
      const QRectF rect(box.left,box.top,box.right-box.left,box.bottom-box.top);
      if(rect.width()<=0||rect.height()<=0)continue;
      if(box.predicted){adas::Obstacle evidence;evidence.predicted=true;evidence.last_observed_s=box.last_observed_s;evidence.track_hits=box.track_hits;
        if(!adas::usable_obstacle(evidence,result.stamp,result.view,now))continue;}
      QColor color(box.color_r,box.color_g,box.color_b);painter.setPen(QPen(color,2,box.predicted?Qt::DashLine:Qt::SolidLine));painter.setBrush(Qt::NoBrush);painter.drawRect(rect);
      auto text=QString::fromUtf8(box.label.c_str());
      if(box.track_id>=0)text+=QStringLiteral(" #%1").arg(qulonglong(box.track_id)&0xffffffffULL);
      if(box.predicted)text+=QStringLiteral(" 跟踪预测");
      auto extent=painter.fontMetrics().boundingRect(text);QRectF label(rect.x(),std::max(0.0,rect.y()-extent.height()-4),extent.width()+6,extent.height()+4);
      painter.fillRect(label,QColor(0,0,0,190));painter.drawText(label.adjusted(3,0,0,0),Qt::AlignVCenter|Qt::AlignLeft,text);
    }
  };
  for(int v=0;v<views;++v) {
    const auto rect=adas_mode?adas::view_rect(displayed_.layout,static_cast<adas::ViewId>(v),displayed_.width,displayed_.height):adas::ViewRect{0,0,displayed_.width,displayed_.height};
    painter.save();painter.translate(rect.x,rect.y);painter.setClipRect(0,0,rect.w,rect.h);
    if(adas_mode) {
      const auto &result=views_[v];
      if(fresh(result.stamp,v==0?engine_.front_timeout_s:engine_.side_timeout_s))draw_boxes(result);
      if(fresh(vehicle_views_[v].stamp,v==0?engine_.front_timeout_s:engine_.side_timeout_s))draw_boxes(vehicle_views_[v]);
      if(fresh(roadblock_views_[v].stamp,v==0?engine_.front_timeout_s:engine_.side_timeout_s))draw_boxes(roadblock_views_[v]);
      if(crosswalk_views_[v].observation.crosswalk_status==adas::Status::Valid&&
         fresh(crosswalk_views_[v].stamp,v==0?engine_.front_timeout_s:engine_.side_timeout_s))draw_boxes(crosswalk_views_[v]);
      const int tag_height=qRound(21*text_scale);
      QFont font=painter.font();font.setPixelSize(qRound(12*text_scale));painter.setFont(font);
      const QRect tag(qRound(6*text_scale),rect.h-tag_height-qRound(5*text_scale),qRound(45*text_scale),tag_height);
      painter.fillRect(tag,QColor(0,0,0,150));painter.setPen(Qt::white);painter.drawText(tag,Qt::AlignCenter,names[v]);
      if(v==0) {
        if(last_geometry_.status==adas::Status::Valid&&fresh(last_geometry_.stamp,engine_.front_timeout_s)) {
          const auto &lane=last_geometry_.lane;
          auto boundary=[&](bool valid,adas::LineType type,float top,float bottom){
            if(!valid)return;
            const auto color=type==adas::LineType::Solid?QColor(255,210,50):type==adas::LineType::Dashed?QColor(80,255,140):QColor(150,160,170);
            const auto style=type==adas::LineType::Solid?Qt::SolidLine:type==adas::LineType::Dashed?Qt::DashLine:Qt::DotLine;
            painter.setPen(QPen(color,3,style));
            painter.drawLine(QPointF(top,lane.top_y),QPointF(bottom,lane.bottom_y));
          };
          boundary(lane.left_valid,lane.left_type,lane.left_top_x,lane.left_x);
          boundary(lane.right_valid,lane.right_type,lane.right_top_x,lane.right_x);
        }
        if(decision_.local_plan.valid&&decision_.local_plan.point_count) {
          QPolygonF path;
          for(unsigned i=0;i<decision_.local_plan.point_count&&i<decision_.local_plan.points.size();++i)
            path<<QPointF(decision_.local_plan.points[i].x*rect.w,decision_.local_plan.points[i].y*rect.h);
          painter.setPen(QPen(QColor(80,210,255,210),4));painter.setBrush(Qt::NoBrush);painter.drawPolyline(path);
        }
        painter.setPen(QPen(QColor(70,180,255,130),1,Qt::DashLine));painter.setBrush(QColor(70,160,255,18));
        adas::LaneObservation road{last_geometry_.stamp,last_geometry_.status,last_geometry_.width,last_geometry_.height,last_geometry_.lane,last_geometry_.process_ms};
        if(!fresh(road.stamp,engine_.front_timeout_s)||road.width!=rect.w||road.height!=rect.h)road.status=adas::Status::Missing;
        QPolygonF corridor;
        for(const auto &point:adas::CollisionGeometry::near_polygon(road))corridor<<QPointF(point.x*rect.w,point.y*rect.h);
        painter.drawPolygon(corridor);
        painter.save();painter.translate(rect.w*.5,rect.h*.78);const double scale=rect.h/360.0;painter.scale(scale,scale);
        const bool blink=(int(adas::monotonic_seconds()*2)%2)==0;
        if(decision_.arrow!=adas::Direction::None) {
          if(decision_.arrow==adas::Direction::Left)painter.rotate(-90);
          if(decision_.arrow==adas::Direction::Right)painter.rotate(90);
          QPolygonF arrow;arrow<<QPointF(0,-35)<<QPointF(24,-8)<<QPointF(9,-8)<<QPointF(9,26)<<QPointF(-9,26)<<QPointF(-9,-8)<<QPointF(-24,-8);
          painter.setPen(QPen(Qt::white,2));painter.setBrush(decision_.any_alarm()?QColor(255,65,65):QColor(45,235,130));painter.drawPolygon(arrow);
        } else {painter.setBrush(QColor(210,40,40,220));painter.setPen(QPen(Qt::white,2));painter.drawRoundedRect(QRect(-42,-18,84,36),6,6);QFont stop_font=painter.font();stop_font.setPixelSize(20);stop_font.setBold(true);painter.setFont(stop_font);painter.drawText(QRect(-42,-18,84,36),Qt::AlignCenter,QStringLiteral("停车"));}
        painter.restore();
        painter.setPen(Qt::NoPen);
        const int led_y=rect.h-65;
        for(int i=0;i<3;++i){painter.setBrush((decision_.led_mask&(1<<i))?QColor(80,255,140):QColor(60,65,70));painter.drawEllipse(QPoint(18+i*22,led_y),7,7);}
        if(ego_.signal!=adas::TurnSignal::None&&blink){painter.setPen(QColor(255,195,30));painter.drawText(QRect(85,led_y-12,150,24),Qt::AlignVCenter,ego_.signal==adas::TurnSignal::Left?QStringLiteral("◀ 左转向"):QStringLiteral("右转向 ▶"));}
      }
    }
    painter.restore();
  }
  if(adas_mode) {
    const auto px=[&](double size){return qRound(size*text_scale);};
    const int width=output.width(),header_height=px(49),footer_height=px(24);
    const bool alarm=decision_.any_alarm();
    painter.fillRect(QRect(0,0,width,header_height),QColor(4,18,30,225));
    QFont font=painter.font();font.setPixelSize(px(18));font.setBold(true);painter.setFont(font);
    painter.setPen(decision_.auto_driving?QColor(95,250,165):QColor(255,215,130));
    painter.drawText(QRect(px(10),0,width-px(180),px(27)),Qt::AlignLeft|Qt::AlignVCenter,
      motion_text(decision_,ego_)+QStringLiteral(" · ")+direction_text(decision_.arrow));
    font.setPixelSize(px(13));painter.setFont(font);painter.setPen(Qt::white);
    const auto speed=decision_.speed_valid?QStringLiteral("仿真 %1 km/h").arg(decision_.speed_kmh,0,'f',1):QStringLiteral("仿真速度无效");
    painter.drawText(QRect(width-px(170),0,px(160),px(27)),Qt::AlignRight|Qt::AlignVCenter,speed);
    font.setPixelSize(px(14));painter.setFont(font);
    QString navigation=navigation_text(ego_);
    if(ego_.force_lane_change!=adas::TurnSignal::None)
      navigation+=ego_.force_lane_change==adas::TurnSignal::Left?QStringLiteral(" · 强制左变道"):QStringLiteral(" · 强制右变道");
    else if(ego_.signal!=adas::TurnSignal::None)
      navigation+=ego_.signal==adas::TurnSignal::Left?QStringLiteral(" · 左转向"):QStringLiteral(" · 右转向");
    painter.drawText(QRect(px(10),px(25),width-px(115),px(23)),Qt::AlignLeft|Qt::AlignVCenter,navigation);
    const QColor light_color=decision_.light==adas::LightState::Red?QColor(255,65,65):
      decision_.light==adas::LightState::Green?QColor(80,245,125):
      decision_.light==adas::LightState::Yellow?QColor(255,210,45):QColor(165,175,185);
    painter.setPen(Qt::NoPen);painter.setBrush(light_color);
    painter.drawEllipse(QPointF(width-px(85),px(36)),px(5),px(5));painter.setPen(light_color);
    painter.drawText(QRect(width-px(75),px(25),px(65),px(23)),Qt::AlignRight|Qt::AlignVCenter,QString::fromUtf8(adas::light_text(decision_.light)));

    // All alarms share the same visible banner. No alarm text is elided or
    // relegated to the sidebar; multiple alarms get their own compact cells.
    const auto alerts=alarm_messages(decision_);
    if(!alerts.isEmpty()) {
      const int columns=alerts.size()>1?2:1,rows=(alerts.size()+columns-1)/columns;
      const int row_height=px(alerts.size()>2?25:32);
      const QRect banner(px(5),header_height+px(3),width-px(10),rows*row_height);
      painter.fillRect(banner,QColor(180,0,12,235));painter.setBrush(Qt::NoBrush);painter.setPen(QPen(QColor(255,140,140),px(1)));painter.drawRect(banner);
      font.setPixelSize(px(alerts.size()>1?17:22));font.setBold(true);painter.setFont(font);painter.setPen(Qt::white);
      for(int i=0;i<alerts.size();++i) {
        const QRect cell(banner.x()+(i%columns)*banner.width()/columns+px(7),banner.y()+(i/columns)*row_height,banner.width()/columns-px(14),row_height);
        painter.drawText(cell,Qt::AlignLeft|Qt::AlignVCenter,QStringLiteral("! ")+alerts[i]);
      }
    }
    painter.fillRect(QRect(0,output.height()-footer_height,width,footer_height),QColor(4,18,30,220));
    font.setPixelSize(px(12));font.setBold(false);painter.setFont(font);painter.setPen(QColor(210,225,235));
    const auto status=QString::fromStdString(decision_.status_text);
    const QRect status_rect(px(9),output.height()-footer_height,width-px(175),footer_height);
    painter.drawText(status_rect,Qt::AlignLeft|Qt::AlignVCenter,painter.fontMetrics().elidedText(status,Qt::ElideRight,status_rect.width()));
    painter.drawText(QRect(width-px(160),output.height()-footer_height,px(151),footer_height),Qt::AlignRight|Qt::AlignVCenter,input_fps_text(capture_hz_));
    painter.setPen(QPen(alarm?QColor(255,55,55):QColor(70,125,155),px(alarm?3:1)));painter.setBrush(Qt::NoBrush);painter.drawRect(output.rect().adjusted(3,3,-3,-3));
  }
  painter.end();rendered_=output;
  ui_->Label->setPixmap(output.scaled(ui_->Label->size(),Qt::KeepAspectRatio,Qt::FastTransformation));
  if(adas_mode) {
    const double rendered_at=adas::monotonic_seconds();
    auto record=[&](const adas::Stamp &stamp,uint64_t &last,std::vector<double> &samples) {
      if(stamp.generation==gen&&stamp.sequence>last) {
        last=stamp.sequence;
        if(samples.size()<60000)samples.push_back((rendered_at-stamp.receive_s)*1000);
      }
    };
    if(views_[0].observation.traffic_status==adas::Status::Valid)record(views_[0].stamp,last_latency_sequence_,detection_latency_);
    if(last_geometry_.status==adas::Status::Valid)record(last_geometry_.stamp,last_geometry_latency_sequence_,geometry_latency_);
    if(vehicle_views_[0].observation.vehicle_status==adas::Status::Valid)record(vehicle_views_[0].stamp,last_vehicle_latency_sequence_,vehicle_latency_);
  }
  if(displayed_.stamp.sequence!=last_presented_sequence_){++shown_;last_presented_sequence_=displayed_.stamp.sequence;}
 }
void AdasController::refresh_stats(double now) {
  const double dt=now-last_stats_s_;if(dt<=0)return;
  if(accepting_live_.load()&&device_->isRunning())sync_live_source();
  const auto received=captured_.load()+pool_dropped_.load();capture_hz_=(received-last_captured_)/dt;preview_hz_=(shown_-last_shown_)/dt;
  lane_hz_=(geometry_count_-previous_geometry_count_)/dt;
  for(int v=0;v<4;++v){
    detection_hz_[v]=(detection_count_[v]-previous_detection_count_[v])/dt;previous_detection_count_[v]=detection_count_[v];
    vehicle_hz_[v]=(vehicle_count_[v]-previous_vehicle_count_[v])/dt;previous_vehicle_count_[v]=vehicle_count_[v];
    crosswalk_hz_[v]=(crosswalk_count_[v]-previous_crosswalk_count_[v])/dt;previous_crosswalk_count_[v]=crosswalk_count_[v];
  }
  last_stats_s_=now;last_captured_=received;last_shown_=shown_;previous_geometry_count_=geometry_count_;
  ui_->frame->setText(QStringLiteral("%1 fps").arg(capture_hz_,0,'f',1));ui_->total_frame->setText(QString::number(received));

}
void AdasController::write_report(const QString &path) const {
  if(path.isEmpty())return;
  auto percentile=[](std::vector<double> values){if(values.empty())return -1.;std::sort(values.begin(),values.end());return values[static_cast<size_t>((values.size()-1)*.95)];};
  const double duration=std::max(.001,adas::monotonic_seconds()-started_s_);
  QJsonObject report;report["duration_s"]=duration;report["input"]=QStringLiteral("fpga_pcie");
  report["input_layout"]=ego_.input_layout==adas::Layout::Single?QStringLiteral("single"):QStringLiteral("mosaic");
  report["lane_change_allowed"]=decision_.lane_change_allowed;report["arrow"]=int(decision_.arrow);
  report["alarm_illegal_lane_change"]=decision_.alarm_illegal_lane_change;
  report["decision_hint"]=QString::fromStdString(decision_.hint_text);
  report["left_line_type"]=int(last_geometry_.lane.left_type);report["right_line_type"]=int(last_geometry_.lane.right_type);
  report["navigation_source"]=QStringLiteral("manual");report["navigation_detected"]=ego_.manual_nav_enabled;
  report["navigation_direction"]=int(ego_.manual_nav_enabled?ego_.manual_nav:adas::Direction::None);
  report["mode"]=int(mode_);
  report["simulation_paused"]=ego_.paused;
  report["captured_frames"]=double(captured_.load());report["rendered_frames"]=double(shown_);
  const auto received=captured_.load()+pool_dropped_.load();
  report["received_frames"]=double(received);report["input_fps"]=capture_hz_;
  report["input_fps_average"]=received/duration;
  report["capture_fps_average"]=captured_.load()/duration;report["render_fps_average"]=shown_/duration;
  report["lane_backend"]=QStringLiteral("classical_cpu");report["lane_cpu_hz_average"]=geometry_count_/duration;
  report["lane_npu_hz_average"]=0.;report["front_receive_to_hud_p95_ms"]=percentile(detection_latency_);
  report["geometry_receive_to_hud_p95_ms"]=percentile(geometry_latency_);
  report["vehicle_receive_to_hud_p95_ms"]=percentile(vehicle_latency_);
  report["pool_dropped"]=double(pool_dropped_.load());report["npu_overwritten"]=double(detector_->dropped_frames());report["result_overwritten"]=double(result_overwritten_.load());
  QJsonArray counts,rates;for(int v=0;v<4;++v){counts.append(double(detection_count_[v]));rates.append(detection_count_[v]/duration);}report["detections_per_view"]=counts;report["npu_hz_average"]=rates;
  QJsonArray vehicle_rates;for(auto count:vehicle_count_)vehicle_rates.append(count/duration);report["vehicle_hz_average"]=vehicle_rates;
  report["traffic_ready"]=views_[0].observation.traffic_status==adas::Status::Valid;
  report["dual_model_profile"]=engine_.separate_crosswalk_source&&engine_.required_obstacle_components==adas::ObjectObservation::Combined;
  report["required_obstacle_components"]=int(engine_.required_obstacle_components);
  report["separate_crosswalk_source"]=engine_.separate_crosswalk_source;
  report["crosswalk_ready"]=crosswalk_views_[0].observation.crosswalk_status==adas::Status::Valid;
  report["crosswalk_status"]=int(crosswalk_views_[0].observation.crosswalk_status);
  report["crosswalk_sequence"]=double(crosswalk_views_[0].stamp.sequence);
  report["crosswalk_generation"]=double(crosswalk_views_[0].stamp.generation);
  report["crosswalk_receive_s"]=crosswalk_views_[0].stamp.receive_s;
  report["crosswalk_source_s"]=crosswalk_views_[0].stamp.source_s;
  report["crosswalk_static_image"]=crosswalk_views_[0].stamp.static_image;
  report["vehicle_ready"]=vehicle_views_[0].observation.vehicle_status==adas::Status::Valid;
  report["roadblock_ready"]=roadblock_views_[0].observation.roadblock_status==adas::Status::Valid;report["calibration_valid"]=calibration_.valid();
  report["live_source_known"]=live_source_known_;
  report["generation"]=double(generation_.load());
  const double wall=adas::monotonic_seconds();
  QJsonArray ages,vehicle_ages,crosswalk_ages,crosswalk_sequences,crosswalk_counts,crosswalk_rates,crosswalk_statuses;
  for(int v=0;v<4;++v) {
    ages.append(views_[v].stamp.generation==generation_.load()?(wall-views_[v].stamp.receive_s)*1000:-1.);
    vehicle_ages.append(vehicle_views_[v].stamp.generation==generation_.load()?(wall-vehicle_views_[v].stamp.receive_s)*1000:-1.);
    const auto &crosswalk=crosswalk_views_[v];
    crosswalk_ages.append(crosswalk.stamp.generation==generation_.load()&&crosswalk.stamp.sequence?(wall-crosswalk.stamp.receive_s)*1000:-1.);
    crosswalk_sequences.append(double(crosswalk.stamp.sequence));
    crosswalk_counts.append(double(crosswalk_count_[v]));
    crosswalk_rates.append(crosswalk_count_[v]/duration);
    crosswalk_statuses.append(int(crosswalk.observation.crosswalk_status));
  }
  report["traffic_age_ms"]=ages;report["vehicle_age_ms"]=vehicle_ages;
  report["crosswalk_age_ms"]=crosswalk_ages;report["crosswalk_sequences_per_view"]=crosswalk_sequences;
  report["crosswalk_updates_per_view"]=crosswalk_counts;report["crosswalk_hz_average"]=crosswalk_rates;
  report["crosswalk_status_per_view"]=crosswalk_statuses;
  report["front_last_infer_ms"]=views_[0].observation.infer_ms;
  report["vehicle_last_infer_ms"]=vehicle_views_[0].observation.infer_ms;
  report["crosswalk_last_infer_ms"]=crosswalk_views_[0].observation.infer_ms;
  report["geometry_last_process_ms"]=last_geometry_.process_ms;
  report["cpu_lane_overwritten"]=double(geometry_->dropped_frames());
  report["brake_now"]=decision_.brake_now;report["alarm_collision"]=decision_.alarm_collision;
  report["alarm_red_run"]=decision_.alarm_red_run;report["alarm_overspeed"]=decision_.alarm_overspeed;
  report["alarm_lane_departure"]=decision_.alarm_lane_departure;
  report["auto_driving"]=decision_.auto_driving;report["stop_required"]=decision_.stop_required;
  report["force_drive"]=ego_.force_drive;report["force_lane_change"]=int(ego_.force_lane_change);
  report["light"]=int(decision_.light);
  const QJsonObject hud{{"motion",motion_text(decision_,ego_)},{"navigation",navigation_text(ego_)},
    {"light",QString::fromUtf8(adas::light_text(decision_.light))},
    {"input_fps",input_fps_text(capture_hz_)},{"alerts",QJsonArray::fromStringList(alarm_messages(decision_))}};
  report["hud"]=hud;
  unsigned measured=0,predicted=0;const double report_now=adas::monotonic_seconds();
  for(const auto &view:views_){
    const auto &stamp=view.stamp;const double age=report_now-stamp.receive_s;
    if(stamp.generation!=generation_.load()||!stamp.sequence||age<0||view.observation.traffic_status!=adas::Status::Valid||
        (!stamp.static_image&&age>(view.view==adas::ViewId::Front?engine_.front_timeout_s:engine_.side_timeout_s)))continue;
    for(const auto &object:adas::usable_obstacles(view.observation,report_now))if(object.class_id==1){if(object.predicted)++predicted;else ++measured;}
  }
  report["front_and_side_person_measured"]=int(measured);report["front_and_side_person_predicted"]=int(predicted);
  report["speed_valid"]=decision_.speed_valid;report["speed_kmh"]=decision_.speed_valid?QJsonValue(decision_.speed_kmh):QJsonValue();
  report["speed_source"]=decision_.speed_source==adas::SpeedSource::Simulation?QStringLiteral("simulation"):QStringLiteral("unavailable");
  report["speed_requested_source"]=QStringLiteral("simulation");
  report["input_route"]=QStringLiteral("PC playback/cameras -> FPGA -> existing PCIe/DMA -> ARM");
  report["time_basis"]=QStringLiteral("ARM receive monotonic time; PC media timestamps unavailable");
  report["front_zoom"]=front_only_;report["lane_change_request_s"]=ego_.lane_change_request_s;
  report["lane_status"]=int(last_geometry_.status);
  report["lane_sequence"]=double(last_geometry_.stamp.sequence);
  report["signal_sequence"]=double(last_signal_.stamp.sequence);report["signal_status"]=int(last_signal_.status);
  report["signal_updates"]=double(signal_count_);
  report["lane_error"]=last_geometry_.status==adas::Status::Valid?QString():QString::fromStdString(last_geometry_.speed.reason);
  QJsonObject plan;plan["valid"]=decision_.local_plan.valid;plan["maneuver"]=int(decision_.local_plan.maneuver);
  plan["reason"]=QString::fromStdString(decision_.local_plan.reason);QJsonArray points;
  for(unsigned i=0;i<decision_.local_plan.point_count&&i<decision_.local_plan.points.size();++i)
    points.append(QJsonArray{decision_.local_plan.points[i].x,decision_.local_plan.points[i].y});
  plan["points"]=points;report["local_plan"]=plan;
  report["status"]=QString::fromStdString(decision_.status_text);report["latency_scope"]=QStringLiteral("ARM receive callback to Qt pixmap update; excludes camera/FPGA/display scanout and event confirmation");
  QFile file(path);if(file.open(QIODevice::WriteOnly))file.write(QJsonDocument(report).toJson());
}
void AdasController::screenshot(const QString &path) const {if(!path.isEmpty()&&!rendered_.isNull())rendered_.save(path);}
