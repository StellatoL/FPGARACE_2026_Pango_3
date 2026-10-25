#include "ui/adas_controller.h"
#include "ui/mainwindow.h"
#include "ui_mainwindow.h"
#include "hardware/pango_fun.h"
#include "inference/adas_pipeline.h"
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
QString status_text(adas::Status s) {
  return s==adas::Status::Valid?QStringLiteral("有效"):s==adas::Status::Failed?QStringLiteral("失败"):QStringLiteral("未配置");
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
  detector_=new adas::DetectionWorker(this);
  detector_->set_model_root(project_root());
  connect(detector_,&adas::DetectionWorker::result_ready,this,&AdasController::queue_result,Qt::DirectConnection);
  connect(detector_,&adas::DetectionWorker::geometry_ready,this,&AdasController::queue_geometry,Qt::DirectConnection);
  // Execute this callback in the capture emitter thread before its next read().
  connect(device_,&pango_fun::send_wr_done,this,&AdasController::ingest_live,Qt::DirectConnection);
  setup_controls(); load_profile(); update_settings();
  detector_->set_mode(DetectionMode::Adas); detector_->start();
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
  auto *adas_mode=new QPushButton(QStringLiteral("ADAS 模式"),source);adas_mode->setObjectName("adasMode");l->addWidget(adas_mode);
  connect(adas_mode,&QPushButton::clicked,this,[this](){emit mode_requested(DetectionMode::Adas);});
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
  manual_stop_check_=new QCheckBox(QStringLiteral("人工停车 [空格]"),controls);l->addWidget(manual_stop_check_);
  force_drive_check_=new QCheckBox(QStringLiteral("强制行驶（用于红灯报警）"),controls);l->addWidget(force_drive_check_);
  speed_source_combo_=new QComboBox(controls);speed_source_combo_->addItems({QStringLiteral("仿真速度"),QStringLiteral("外部速度（未接入）")});form->addRow(QStringLiteral("速度来源"),speed_source_combo_);
  speed_spin_=new QDoubleSpinBox(controls);speed_spin_->setRange(0,180);speed_spin_->setValue(0);speed_spin_->setSuffix(" km/h");form->addRow(QStringLiteral("仿真速度"),speed_spin_);
  limit_spin_=new QDoubleSpinBox(controls);limit_spin_->setRange(1,180);limit_spin_->setValue(60);limit_spin_->setSuffix(" km/h");form->addRow(QStringLiteral("限速"),limit_spin_);
  for(auto *combo:{lane_combo_,signal_combo_,force_lane_combo_,nav_combo_,speed_source_combo_})connect(combo,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int){update_ego();});
  for(auto *check:{auto_check_,intersection_check_,manual_stop_check_,force_drive_check_})connect(check,&QCheckBox::toggled,this,[this](bool){update_ego();});
  for(auto *spin:{speed_spin_,limit_spin_})connect(spin,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this](double){update_ego();});
  auto *reset=new QPushButton(QStringLiteral("复位场景与决策"),controls);reset->setObjectName("adasResetDecision");l->addWidget(reset);connect(reset,&QPushButton::clicked,this,&AdasController::reset_decision);
  ui_->infoPanelLayout->insertWidget(0,controls);
  auto *state=group(QStringLiteral("ADAS 状态"),window_,&l);
  decision_label_=wrapping_label(state);alarm_label_=wrapping_label(state);speed_label_=wrapping_label(state);stats_label_=wrapping_label(state);capability_label_=wrapping_label(state);calibration_label_=wrapping_label(state);
  decision_label_->setObjectName("adasDecisionStatus");capability_label_->setObjectName("adasCapabilityStatus");speed_label_->setObjectName("adasSpeedStatus");
  for(auto *label:{decision_label_,alarm_label_,speed_label_,stats_label_,capability_label_,calibration_label_})l->addWidget(label);
  ui_->infoPanelLayout->insertWidget(1,state);
  auto *calib=group(QStringLiteral("标定与感知区域"),window_,&l);
  auto *calibrate_button=new QPushButton(QStringLiteral("前视路面四点标定"),calib);calibrate_button->setObjectName("adasCalibrate");l->addWidget(calibrate_button);connect(calibrate_button,&QPushButton::clicked,this,&AdasController::calibrate);
  auto *roi=new QPushButton(QStringLiteral("各车道信号灯 ROI"),calib);l->addWidget(roi);connect(roi,&QPushButton::clicked,this,&AdasController::edit_rois);
  auto *clear=new QPushButton(QStringLiteral("清除当前标定"),calib);l->addWidget(clear);connect(clear,&QPushButton::clicked,this,[this](){calibration_.reset();QFile::remove(project_root()+"/config/calibrations/"+profile_key()+".yml");update_settings();advance_generation();});
  ui_->controlPanelLayout->insertWidget(1,calib);
  update_ego();
  lane_combo_->setObjectName("adasLane");signal_combo_->setObjectName("adasTurnSignal");force_lane_combo_->setObjectName("adasForceLane");
  nav_combo_->setObjectName("adasNavigation");manual_stop_check_->setObjectName("adasManualStop");force_drive_check_->setObjectName("adasForceDrive");
  speed_source_combo_->setObjectName("adasSpeedSource");speed_spin_->setObjectName("adasManualSpeed");limit_spin_->setObjectName("adasSpeedLimit");
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
}
void AdasController::advance_generation(bool keep_image) {
  const auto gen=++generation_;engine_.reset(gen);
  display_mailbox_.take();
  {
    std::lock_guard<std::mutex> lock(results_mutex_);
    pending_results_={};pending_result_ready_={};pending_geometry_={};pending_geometry_ready_={};
    pending_legacy_={};pending_legacy_ready_=false;
  }
  views_={};vehicle_views_={};roadblock_views_={};last_geometry_={};last_navigation_={};last_signal_={};legacy_result_={};decision_={};
  // A retained preview never acquires a new capture stamp or re-enters inference.
  if(!keep_image){displayed_={};rendered_=QPixmap();ui_->Label->clear();}
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
  advance_generation(false);return true;
}
bool AdasController::shutdown() {
  if(closing_)return true;
  if(!stop_live())return false;
  closing_=true;timer_.stop();
  disconnect(device_,nullptr,this,nullptr);
  detector_->stop();detector_->wait();
  qApp->removeEventFilter(this);return true;
}
bool AdasController::prepare_live() {
  if(device_->isRunning()&&!accepting_live_) {
    if(!stop_live())return false;
  }
  // Reopening capture cannot reuse another capture session's source identity.
  accepting_live_=false;live_source_known_=false;
  update_ego();
  sync_live_source();load_profile();update_settings();advance_generation(false);accepting_live_=true;
  source_label_->setText(QStringLiteral("FPGA → PCIe · 1280×720 RGB565\n时间基准：ARM 接收单调时间"));return true;
}
bool AdasController::start_file(const QString &path) {
  Q_UNUSED(path);
  const auto message=QStringLiteral("ARM 不直接打开素材。请在电脑播放 U 盘图片/视频，经 HDMI 输入 FPGA，再使用原有采集按钮。");
  source_label_->setText(message);window_->statusBar()->showMessage(message,10000);
  return false;
}
void AdasController::set_layout(adas::Layout layout) {
  if(live_source_known_)layout=ui_->radioHDMI->isChecked()?adas::Layout::Single:adas::Layout::Mosaic;
  const int value=static_cast<int>(layout);
  if(layout_.load()==value)return;
  const bool was_live=accepting_live_.exchange(false);
  layout_=value;{QSignalBlocker b(mosaic_check_);mosaic_check_->setChecked(layout==adas::Layout::Mosaic);}
  update_ego();load_profile();update_settings();advance_generation(false);
  accepting_live_=was_live;
}
void AdasController::set_mode(DetectionMode mode) {
  if(mode_==mode)return;
  mode_=mode;detector_->set_mode(mode);
  advance_generation();
}
void AdasController::set_geometry_enabled(bool enabled) {
  geometry_enabled_=enabled;advance_generation();
}
void AdasController::physical_source_changed() {
  const bool was_live=accepting_live_.exchange(false);
  // A successful select write is not yet evidence that a complete frame from
  // that source has reached DMA. Confirm through the existing status read only
  // after the established settling interval; preserve the original buttons.
  live_source_known_=false;
  discard_live_before_=adas::monotonic_seconds()+.25;
  load_profile();update_settings();advance_generation(false);
  source_label_->setText(QStringLiteral("FPGA 视频源切换中：等待状态确认与新帧"));
  accepting_live_=was_live;
}
void AdasController::sync_live_source() {
  if(device_->dev_fd<0) {
    if(live_source_known_) {
      live_source_known_=false;
      load_profile();update_settings();advance_generation(false);
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
  load_profile();update_settings();advance_generation(false);
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
  nav_roi_=read_roi("navigation",{0,0,.32,.30});
  for(int i=0;i<3;++i)light_rois_[i]=read_roi("signal_"+QString::number(i),{0,0,1,.5});
}
void AdasController::update_settings() {
  detector_->configure(calibration_,light_rois_,nav_roi_);
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
  ego_.use_visual_speed=false;ego_.speed_source=speed_source_combo_->currentIndex()==0?adas::SpeedSource::Simulation:adas::SpeedSource::External;
  ego_.manual_speed_kmh=float(speed_spin_->value());ego_.speed_limit=float(limit_spin_->value());
  ego_.paused=false;speed_spin_->setEnabled(ego_.speed_source==adas::SpeedSource::Simulation);
  refresh_turn_request(old_signal!=ego_.signal||old_force!=ego_.force_lane_change||old_manual!=ego_.manual_nav_enabled||old_nav!=ego_.manual_nav);
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
  advance_generation(false);
}
// Direct worker callbacks only replace bounded latest metadata. All decisions,
// counters, and widget access stay on the GUI thread in drain_results().
void AdasController::queue_result(VisionFrameResult result) {
  std::lock_guard<std::mutex> lock(results_mutex_);
  if(result.stamp.generation!=generation_.load())return;
  if(result.mode!=DetectionMode::Adas) {
    if(!pending_legacy_ready_||result.stamp.sequence>pending_legacy_.stamp.sequence) {
      if(pending_legacy_ready_)++result_overwritten_;
      pending_legacy_=std::move(result);pending_legacy_ready_=true;
    }
    return;
  }
  const int view=int(result.view);if(view<0||view>3)return;
  for(int component=0;component<4;++component) {
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
  if(result.stamp.generation!=generation_.load())return;
  for(int component=0;component<2;++component) {
    const unsigned bit=1u<<component;if(!(result.components&bit))continue;
    auto &slot=pending_geometry_[component];auto &ready=pending_geometry_ready_[component];
    if(ready&&result.stamp.sequence<=slot.stamp.sequence)continue;
    if(ready)++result_overwritten_;
    slot=result;slot.components=bit;ready=true;
  }
}
void AdasController::drain_results() {
  std::array<std::array<VisionFrameResult,4>,4> results{};
  std::array<std::array<bool,4>,4> ready{};
  std::array<adas::GeometryObservation,2> geometry{};std::array<bool,2> geometry_ready{};
  VisionFrameResult legacy;bool legacy_ready=false;
  {
    std::lock_guard<std::mutex> lock(results_mutex_);
    results.swap(pending_results_);ready.swap(pending_result_ready_);
    geometry.swap(pending_geometry_);geometry_ready.swap(pending_geometry_ready_);
    std::swap(legacy,pending_legacy_);std::swap(legacy_ready,pending_legacy_ready_);
  }
  for(int view=0;view<4;++view)for(int component=0;component<4;++component)
    if(ready[view][component])accept_result(std::move(results[view][component]));
  for(int component=0;component<2;++component)if(geometry_ready[component])accept_geometry(std::move(geometry[component]));
  if(legacy_ready)accept_result(std::move(legacy));
}
void AdasController::accept_result(VisionFrameResult result) {
  if(closing_||result.stamp.generation!=generation_.load()||result.mode!=mode_)return;
  const int v=static_cast<int>(result.view);if(v<0||v>3)return;
  if(result.observation.stamp.generation!=result.stamp.generation||result.observation.stamp.sequence!=result.stamp.sequence)return;
  if(engine_.generation()!=generation_.load())engine_.reset(generation_.load());
  if(mode_==DetectionMode::Adas) {
    auto component_result=[&](unsigned component) {
      auto split=result;split.observation.components=component;
      split.overlays.erase(std::remove_if(split.overlays.begin(),split.overlays.end(),[&](const OverlayBoxResult &box) {
        return component==adas::ObjectObservation::Vehicle?box.class_id!=10:
          component==adas::ObjectObservation::Roadblock?box.class_id!=11:box.class_id==10||box.class_id==11;
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
    const bool signal=v==0&&((result.observation.components&adas::ObjectObservation::Signal)||
      ((result.observation.components&adas::ObjectObservation::Traffic)&&result.observation.signal_observation));
    if(signal&&result.stamp.sequence>last_signal_.stamp.sequence) {
      const auto &p=result.observation;
      const auto status=p.signal_status==adas::Status::Missing&&(p.components&adas::ObjectObservation::Traffic)?p.traffic_status:p.signal_status;
      last_signal_={result.stamp,status,p.light,p.has_lane_lights,p.lane_lights};if(status==adas::Status::Valid)++signal_count_;
    }
    if(result.observation.components&adas::ObjectObservation::Signal)accepted|=adas::ObjectObservation::Signal;
    if(accepted){result.observation.components=accepted;engine_.ingest(result.observation,adas::monotonic_seconds());}
  } else if(result.stamp.sequence>legacy_result_.stamp.sequence) {
    legacy_result_=result;++detection_count_[0];
  }
}
void AdasController::accept_geometry(adas::GeometryObservation result) {
  if(closing_||mode_!=DetectionMode::Adas||result.stamp.generation!=generation_.load())return;
  if(engine_.generation()!=generation_.load())engine_.reset(generation_.load());
  unsigned accepted=0;
  if(geometry_enabled_&&(result.components&adas::GeometryObservation::Lane)&&result.stamp.sequence>last_geometry_.stamp.sequence) {
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
  switch(key->key()) {
  case Qt::Key_1:case Qt::Key_2:case Qt::Key_3:lane_combo_->setCurrentIndex(key->key()-Qt::Key_1);break;
  case Qt::Key_A:signal_combo_->setCurrentIndex(signal_combo_->currentIndex()==1?0:1);break;
  case Qt::Key_D:signal_combo_->setCurrentIndex(signal_combo_->currentIndex()==2?0:2);break;
  case Qt::Key_J:force_lane_combo_->setCurrentIndex(force_lane_combo_->currentIndex()==1?0:1);break;
  case Qt::Key_L:force_lane_combo_->setCurrentIndex(force_lane_combo_->currentIndex()==2?0:2);break;
  case Qt::Key_Space:manual_stop_check_->toggle();break;
  case Qt::Key_N:nav_combo_->setCurrentIndex((nav_combo_->currentIndex()+1)%4);break;
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
  render(now);refresh_labels(now);
  if(wall-last_stats_s_>=1)refresh_stats(wall);
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
      QColor color(box.color_r,box.color_g,box.color_b);painter.setPen(QPen(color,2));painter.setBrush(Qt::NoBrush);painter.drawRect(rect);
      auto text=QString::fromUtf8(box.label.c_str());
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
      const int header_height=qRound(24*text_scale);
      painter.fillRect(QRect(0,0,rect.w,header_height),QColor(0,0,0,160));painter.setPen(Qt::white);
      QFont font=painter.font();font.setPixelSize(qRound(13*text_scale));painter.setFont(font);
      const bool current=fresh(result.stamp,v==0?engine_.front_timeout_s:engine_.side_timeout_s);
      painter.drawText(QRect(8,1,rect.w-16,header_height-2),Qt::AlignLeft|Qt::AlignVCenter,
        QStringLiteral("%1 · %2 Hz · %3").arg(names[v]).arg(detection_hz_[v],0,'f',1)
          .arg(current?QStringLiteral("%1 ms").arg(std::max(0.0,now-result.stamp.receive_s)*1000,0,'f',0):QStringLiteral("感知不可用/过期")));
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
          painter.setPen(QPen(QColor(100,220,255,180),1,Qt::DashLine));painter.setBrush(Qt::NoBrush);
          painter.drawRect(QRectF(nav_roi_.x*rect.w,nav_roi_.y*rect.h,nav_roi_.w*rect.w,nav_roi_.h*rect.h));
        }
        if(last_navigation_.status==adas::Status::Valid&&fresh(last_navigation_.stamp,engine_.front_timeout_s)&&last_navigation_.nav.detected) {
          const auto &b=last_navigation_.nav.box;painter.setPen(QPen(Qt::cyan,2));painter.setBrush(Qt::NoBrush);painter.drawRect(QRectF(b.left,b.top,b.right-b.left,b.bottom-b.top));
        }
        if(decision_.local_plan.valid&&decision_.local_plan.point_count) {
          QPolygonF path;
          for(unsigned i=0;i<decision_.local_plan.point_count&&i<decision_.local_plan.points.size();++i)
            path<<QPointF(decision_.local_plan.points[i].x*rect.w,decision_.local_plan.points[i].y*rect.h);
          painter.setPen(QPen(QColor(80,210,255,210),4));painter.setBrush(Qt::NoBrush);painter.drawPolyline(path);
        }
        painter.setPen(QPen(QColor(70,180,255,130),1,Qt::DashLine));painter.setBrush(QColor(70,160,255,18));
        QPolygonF corridor;corridor<<QPointF(rect.w*.35,rect.h)<<QPointF(rect.w*.45,rect.h*.58)<<QPointF(rect.w*.55,rect.h*.58)<<QPointF(rect.w*.65,rect.h);
        painter.drawPolygon(corridor);
        painter.save();painter.translate(rect.w*.5,rect.h*.78);const double scale=rect.h/360.0;painter.scale(scale,scale);
        const bool blink=(int(adas::monotonic_seconds()*2)%2)==0;
        if(decision_.arrow!=adas::Direction::None) {
          if(decision_.arrow==adas::Direction::Left)painter.rotate(-90);
          if(decision_.arrow==adas::Direction::Right)painter.rotate(90);
          QPolygonF arrow;arrow<<QPointF(0,-35)<<QPointF(24,-8)<<QPointF(9,-8)<<QPointF(9,26)<<QPointF(-9,26)<<QPointF(-9,-8)<<QPointF(-24,-8);
          painter.setPen(QPen(Qt::white,2));painter.setBrush(decision_.any_alarm()?QColor(255,65,65):QColor(45,235,130));painter.drawPolygon(arrow);
        } else {painter.setBrush(QColor(210,40,40,220));painter.setPen(QPen(Qt::white,2));painter.drawRoundedRect(QRect(-42,-18,84,36),6,6);painter.drawText(QRect(-42,-18,84,36),Qt::AlignCenter,QStringLiteral("STOP"));}
        painter.restore();
        painter.setPen(Qt::NoPen);
        const int led_y=rect.h-65;
        for(int i=0;i<3;++i){painter.setBrush((decision_.led_mask&(1<<i))?QColor(80,255,140):QColor(60,65,70));painter.drawEllipse(QPoint(18+i*22,led_y),7,7);}
        if(ego_.signal!=adas::TurnSignal::None&&blink){painter.setPen(QColor(255,195,30));painter.drawText(QRect(85,led_y-12,150,24),Qt::AlignVCenter,ego_.signal==adas::TurnSignal::Left?QStringLiteral("◀ 左转向"):QStringLiteral("右转向 ▶"));}
      }
    } else if(mode_!=DetectionMode::RawVideo&&fresh(legacy_result_.stamp,engine_.front_timeout_s)) {
      draw_boxes(legacy_result_);
      const auto &marker=legacy_result_.alert_marker;
      if(marker.visible){painter.setPen(QPen(QColor(marker.stroke_r,marker.stroke_g,marker.stroke_b),3));painter.setBrush(QColor(marker.fill_r,marker.fill_g,marker.fill_b));painter.drawEllipse(QPoint(marker.center_x,marker.center_y),marker.radius,marker.radius);painter.drawText(QRect(marker.center_x-marker.radius,marker.center_y-marker.radius,marker.radius*2,marker.radius*2),Qt::AlignCenter,QString::fromStdString(marker.text));}
    }
    painter.restore();
  }
  if(adas_mode) {
    const bool alarm=decision_.any_alarm();painter.setPen(QPen(alarm?QColor(255,55,55):QColor(70,125,155),alarm?6:2));painter.setBrush(Qt::NoBrush);painter.drawRect(output.rect().adjusted(3,3,-3,-3));
    const int height=std::max(output.height()<500?36:46,qRound(30*text_scale));
    painter.fillRect(QRect(0,output.height()-height,output.width(),height),alarm?QColor(140,15,20,225):QColor(4,18,30,215));
    QFont font=painter.font();font.setPixelSize(qRound((output.width()<800?13:18)*text_scale));font.setBold(true);painter.setFont(font);painter.setPen(Qt::white);
    auto speed=decision_.speed_valid?QStringLiteral("%1 km/h").arg(decision_.speed_kmh,0,'f',1):QStringLiteral("速度无效");
    painter.drawText(QRect(10,output.height()-height,output.width()-20,height),Qt::AlignVCenter|Qt::AlignLeft,
      painter.fontMetrics().elidedText(QStringLiteral("%1 | %2 | %3").arg(speed,QString::fromUtf8(adas::light_text(decision_.light)),QString::fromStdString(decision_.status_text)),Qt::ElideRight,output.width()-20));
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
void AdasController::refresh_labels(double now) {
  const uint64_t gen=generation_.load();
  const auto fresh=[&](const adas::Stamp &s,double timeout){return s.generation==gen&&s.sequence&&now>=s.receive_s&&(s.static_image||now-s.receive_s<=timeout);};
  if(mode_!=DetectionMode::Adas) {
    alarm_label_->clear();
    decision_label_->setText(mode_==DetectionMode::RawVideo?QStringLiteral("纯视频 · 不进行推理"):QString::fromStdString(legacy_result_.status_text));
    speed_label_->setText(QStringLiteral("切换到 ADAS 模式使用驾驶辅助"));
    capability_label_->setText(QString::fromStdString(legacy_result_.error_message));return;
  }
  decision_label_->setText(QStringLiteral("%1\n%2\n行为：%3 · %4")
    .arg(QString::fromStdString(decision_.status_text),QString::fromStdString(decision_.hint_text),direction_text(decision_.arrow),decision_.auto_driving?QStringLiteral("模拟行驶"):QStringLiteral("停车")));
  decision_label_->setStyleSheet(decision_.any_alarm()?"color:#ff6464; font-weight:bold;":"color:#8be4ba;");
  QStringList alarms;
  if(decision_.brake_now)alarms<<QStringLiteral("行人危险：紧急停车");
  if(decision_.alarm_collision)alarms<<QStringLiteral("碰撞风险");
  if(decision_.alarm_red_run)alarms<<QStringLiteral("红灯强行通过");
  if(decision_.alarm_illegal_lane_change)alarms<<QStringLiteral("实线强制变道");
  if(decision_.alarm_overspeed)alarms<<QStringLiteral("超过设定限速");
  if(decision_.alarm_lane_departure)alarms<<QStringLiteral("车道偏离 / 压实线");
  alarm_label_->setText(alarms.isEmpty()?QStringLiteral("当前告警：无"):QStringLiteral("当前告警：\n• ")+alarms.join(QStringLiteral("\n• ")));
  const QString speed_source=ego_.speed_source==adas::SpeedSource::Simulation?QStringLiteral("仿真速度"):QStringLiteral("外部速度");
  speed_label_->setText(QStringLiteral("%1：%2\n%3\n距离：%4 · TTC：%5")
    .arg(speed_source,decision_.speed_valid?QStringLiteral("%1 km/h").arg(decision_.speed_kmh,0,'f',1):QStringLiteral("无效"),
    ego_.speed_source==adas::SpeedSource::Simulation?QStringLiteral("仅用于模拟，不代表视觉测量"):QStringLiteral("外部接口未接入或样本过期"),
    decision_.nearest_obstacle_m>=0?QStringLiteral("%1 m").arg(decision_.nearest_obstacle_m,0,'f',1):QStringLiteral("无效"),decision_.ttc_s>0?QStringLiteral("%1 s").arg(decision_.ttc_s,0,'f',1):QStringLiteral("无效")));
  const auto &p=views_[0].observation;
  const auto &vehicle=vehicle_views_[0];
  const auto traffic_status=status_text(p.traffic_status)+(views_[0].stamp.sequence&&!fresh(views_[0].stamp,engine_.front_timeout_s)?QStringLiteral("（数据过期）"):QString());
  const auto &roadblock=roadblock_views_[0];
  const auto roadblock_status=roadblock.observation.roadblock_status==adas::Status::Missing?QStringLiteral("未配置（模型待补）"):
    status_text(roadblock.observation.roadblock_status)+(!fresh(roadblock.stamp,engine_.front_timeout_s)?QStringLiteral("（数据过期）"):QString());
  const auto lane_status=status_text(last_geometry_.status)+(last_geometry_.stamp.sequence&&!fresh(last_geometry_.stamp,engine_.front_timeout_s)?QStringLiteral("（数据过期）"):QString());
  const auto navigation_status=ego_.manual_nav_enabled?QStringLiteral("手动 ")+direction_text(ego_.manual_nav):QStringLiteral("关闭（按 N 切换）");
  const auto signal_status=status_text(last_signal_.status)+(last_signal_.stamp.sequence&&!fresh(last_signal_.stamp,engine_.front_timeout_s)?QStringLiteral("（数据过期）"):QString());
  capability_label_->setText(QStringLiteral("前视交通：%1 · 车辆：%2%3\n路障识别：%4\n%5\n车道模型：%6 · 导航：%7\n%8\n信号灯：%9").arg(traffic_status,status_text(vehicle.observation.vehicle_status),
    vehicle.stamp.sequence&&!fresh(vehicle.stamp,engine_.front_timeout_s)?QStringLiteral("（数据过期）"):QString(),roadblock_status,QString::fromStdString(views_[0].error_message),lane_status,navigation_status,
    !last_geometry_.stamp.sequence?QStringLiteral("等待 FPGA 输入与模型观测"):last_geometry_.status==adas::Status::Valid?QString():QString::fromStdString(last_geometry_.speed.reason),signal_status));
}
void AdasController::refresh_stats(double now) {
  const double dt=now-last_stats_s_;if(dt<=0)return;
  if(accepting_live_.load()&&device_->isRunning())sync_live_source();
  const auto received=captured_.load();capture_hz_=(received-last_captured_)/dt;preview_hz_=(shown_-last_shown_)/dt;
  lane_hz_=(geometry_count_-previous_geometry_count_)/dt;
  for(int v=0;v<4;++v){
    detection_hz_[v]=(detection_count_[v]-previous_detection_count_[v])/dt;previous_detection_count_[v]=detection_count_[v];
    vehicle_hz_[v]=(vehicle_count_[v]-previous_vehicle_count_[v])/dt;previous_vehicle_count_[v]=vehicle_count_[v];
  }
  last_stats_s_=now;last_captured_=received;last_shown_=shown_;previous_geometry_count_=geometry_count_;
  ui_->frame->setText(QStringLiteral("%1 fps").arg(capture_hz_,0,'f',1));ui_->total_frame->setText(QString::number(received));
  stats_label_->setText(QStringLiteral("输入 %1 · 渲染 %2 fps\n车道 NPU %3 Hz · 目标前 %4 Hz\n右/后/左 %5/%6/%7 Hz\n车辆前 %8 Hz\n帧池丢弃 %9 · NPU覆盖 %10\n结果覆盖 %11")
    .arg(capture_hz_,0,'f',1).arg(preview_hz_,0,'f',1).arg(lane_hz_,0,'f',1).arg(detection_hz_[0],0,'f',1)
    .arg(detection_hz_[1],0,'f',1).arg(detection_hz_[2],0,'f',1).arg(detection_hz_[3],0,'f',1)
    .arg(vehicle_hz_[0],0,'f',1).arg(pool_dropped_.load()).arg(detector_->dropped_frames()).arg(result_overwritten_.load()));
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
  report["mode"]=int(mode_);report["legacy_model_ready"]=legacy_result_.model_ready;
  report["legacy_error"]=QString::fromStdString(legacy_result_.error_message);
  report["simulation_paused"]=ego_.paused;
  report["captured_frames"]=double(captured_.load());report["rendered_frames"]=double(shown_);
  report["capture_fps_average"]=captured_.load()/duration;report["render_fps_average"]=shown_/duration;
  report["lane_npu_hz_average"]=geometry_count_/duration;report["front_receive_to_hud_p95_ms"]=percentile(detection_latency_);
  report["geometry_receive_to_hud_p95_ms"]=percentile(geometry_latency_);
  report["vehicle_receive_to_hud_p95_ms"]=percentile(vehicle_latency_);
  report["pool_dropped"]=double(pool_dropped_.load());report["npu_overwritten"]=double(detector_->dropped_frames());report["result_overwritten"]=double(result_overwritten_.load());
  QJsonArray counts,rates;for(int v=0;v<4;++v){counts.append(double(detection_count_[v]));rates.append(detection_count_[v]/duration);}report["detections_per_view"]=counts;report["npu_hz_average"]=rates;
  QJsonArray vehicle_rates;for(auto count:vehicle_count_)vehicle_rates.append(count/duration);report["vehicle_hz_average"]=vehicle_rates;
  report["traffic_ready"]=views_[0].observation.traffic_status==adas::Status::Valid;
  report["vehicle_ready"]=vehicle_views_[0].observation.vehicle_status==adas::Status::Valid;
  report["roadblock_ready"]=roadblock_views_[0].observation.roadblock_status==adas::Status::Valid;report["calibration_valid"]=calibration_.valid();
  report["live_source_known"]=live_source_known_;
  report["generation"]=double(generation_.load());
  const double wall=adas::monotonic_seconds();
  QJsonArray ages,vehicle_ages;
  for(int v=0;v<4;++v) {
    ages.append(views_[v].stamp.generation==generation_.load()?(wall-views_[v].stamp.receive_s)*1000:-1.);
    vehicle_ages.append(vehicle_views_[v].stamp.generation==generation_.load()?(wall-vehicle_views_[v].stamp.receive_s)*1000:-1.);
  }
  report["traffic_age_ms"]=ages;report["vehicle_age_ms"]=vehicle_ages;
  report["front_last_infer_ms"]=views_[0].observation.infer_ms;
  report["vehicle_last_infer_ms"]=vehicle_views_[0].observation.infer_ms;
  report["geometry_last_process_ms"]=last_geometry_.process_ms;
  report["speed_valid"]=decision_.speed_valid;report["speed_kmh"]=decision_.speed_valid?QJsonValue(decision_.speed_kmh):QJsonValue();
  report["speed_source"]=decision_.speed_source==adas::SpeedSource::Simulation?QStringLiteral("simulation"):
    decision_.speed_source==adas::SpeedSource::External?QStringLiteral("external"):QStringLiteral("unavailable");
  report["speed_requested_source"]=ego_.speed_source==adas::SpeedSource::Simulation?QStringLiteral("simulation"):QStringLiteral("external");
  report["input_route"]=QStringLiteral("PC playback/cameras -> FPGA -> existing PCIe/DMA -> ARM");
  report["time_basis"]=QStringLiteral("ARM receive monotonic time; PC media timestamps unavailable");
  report["front_zoom"]=front_only_;report["lane_change_request_s"]=ego_.lane_change_request_s;
  report["lane_status"]=int(last_geometry_.status);report["navigation_status"]=int(last_navigation_.status);
  report["lane_sequence"]=double(last_geometry_.stamp.sequence);report["navigation_sequence"]=double(last_navigation_.stamp.sequence);
  report["signal_sequence"]=double(last_signal_.stamp.sequence);report["signal_status"]=int(last_signal_.status);
  report["navigation_updates"]=double(navigation_count_);report["signal_updates"]=double(signal_count_);
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
