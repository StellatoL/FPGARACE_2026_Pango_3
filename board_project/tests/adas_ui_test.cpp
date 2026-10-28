#include "ui/mainwindow.h"
#include "ui/adas_controller.h"
#include "inference/adas_pipeline.h"
#include <QtTest/QtTest>
#include <QCheckBox>
#include <QLabel>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDoubleSpinBox>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QPushButton>
#include <QTemporaryDir>
#include <limits>

class AdasUiTest : public QObject {
  Q_OBJECT
  static AdasController *control(MainWindow &window) {
    auto *c=window.findChild<AdasController*>();
    // Synthetic safety scenes have a fixed capability profile regardless of
    // whichever verified model bundle is active in the application directory.
    if(c){c->engine_.required_obstacle_components=adas::ObjectObservation::All;c->engine_.separate_crosswalk_source=false;}
    return c;
  }
  static VisionFrameResult scene(AdasController *c,uint64_t sequence,adas::ViewId view=adas::ViewId::Front) {
    VisionFrameResult r;r.mode=DetectionMode::Adas;r.model_ready=true;r.view=view;
    const double now=adas::monotonic_seconds();r.stamp={c->generation_.load(),sequence,now,now,false};
    auto &o=r.observation;o.stamp=r.stamp;o.view=view;o.components=adas::ObjectObservation::All;
    o.width=c->layout_.load()==0?640:1280;o.height=c->layout_.load()==0?360:720;
    o.traffic_status=o.vehicle_status=o.roadblock_status=o.signal_status=adas::Status::Valid;
    return r;
  }
  static adas::GeometryObservation road(AdasController *c,uint64_t sequence,adas::LineType type=adas::LineType::Dashed) {
    const auto p=scene(c,sequence);adas::GeometryObservation g;g.stamp=p.stamp;
    g.components=adas::GeometryObservation::Lane;g.status=adas::Status::Valid;
    g.width=p.observation.width;g.height=p.observation.height;
    auto &l=g.lane;l.left_valid=l.right_valid=true;l.left_type=l.right_type=type;
    l.left_x=g.width*.328125f;l.right_x=g.width*.671875f;
    l.left_top_x=g.width*.4375f;l.right_top_x=g.width*.5625f;l.top_y=g.height*.5f;l.bottom_y=float(g.height);
    return g;
  }
  static VisionFrameResult primary(AdasController *c,uint64_t sequence,adas::ViewId view=adas::ViewId::Front) {
    auto r=scene(c,sequence,view);
    auto &p=r.observation;p.components=adas::ObjectObservation::Combined;
    p.roadblock_status=p.signal_status=adas::Status::Missing;p.signal_observation=false;
    return r;
  }
  static VisionFrameResult crossing(AdasController *c,uint64_t sequence,bool danger=false,
                                    adas::ViewId view=adas::ViewId::Front) {
    auto r=scene(c,sequence,view);auto &p=r.observation;
    p.components=adas::ObjectObservation::Crosswalk;p.signal_observation=false;
    p.traffic_status=p.vehicle_status=p.roadblock_status=p.signal_status=adas::Status::Missing;
    p.crosswalk_status=adas::Status::Valid;p.has_zebra=true;p.person_on_zebra=danger;
    return r;
  }
  static void dual(AdasController *c) {
    c->engine_.required_obstacle_components=adas::ObjectObservation::Combined;
    c->engine_.separate_crosswalk_source=true;
  }
  static QJsonObject model_fixture(const QTemporaryDir &dir,const QString &name,bool traffic) {
    const QByteArray data=QByteArray("UI manifest fixture only: ")+name.toUtf8();
    QFile file(dir.filePath(name));
    if(!file.open(QIODevice::WriteOnly)||file.write(data)!=data.size())return {};
    file.close();
    const auto classes=traffic?QStringLiteral("crosswalk,person,light_red,light_green,light_black,light_yellow"):
      QStringLiteral("person,bicycle,car,motorcycle,airplane,bus,train,truck,boat,traffic light,fire hydrant,stop sign,parking meter,bench,"
        "bird,cat,dog,horse,sheep,cow,elephant,bear,zebra,giraffe,backpack,umbrella,handbag,tie,suitcase,frisbee,skis,snowboard,"
        "sports ball,kite,baseball bat,baseball glove,skateboard,surfboard,tennis racket,bottle,wine glass,cup,fork,knife,spoon,bowl,"
        "banana,apple,sandwich,orange,broccoli,carrot,hot dog,pizza,donut,cake,chair,couch,potted plant,bed,dining table,toilet,"
        "tv,laptop,mouse,remote,keyboard,cell phone,microwave,oven,toaster,sink,refrigerator,book,clock,vase,scissors,teddy bear,hair drier,toothbrush");
    const std::array<std::array<float,6>,3> dimensions=traffic?
      std::array<std::array<float,6>,3>{{{5.64062f,12.39062f,7.85547f,18.5f,10.71875f,28.14062f},
        {63.5f,9.07812f,15.92188f,41.3125f,25.48438f,69.9375f},{218.375f,16.32812f,49.90625f,100.3125f,612.5f,90.f}}}:
      std::array<std::array<float,6>,3>{{{10,13,16,30,33,23},{30,61,62,45,59,119},{116,90,156,198,373,326}}};
    QJsonArray anchors;for(const auto &head:dimensions){QJsonArray row;for(float value:head)row.append(value);anchors.append(row);}
    return {{"file",name},{"sha256",QString::fromLatin1(QCryptographicHash::hash(data,QCryptographicHash::Sha256).toHex())},
      {"quantization","int8"},{"runtime_api_prefix","2.3.2 ("},{"classes",QJsonArray::fromStringList(classes.split(','))},
      {"output",traffic?"yolov5_raw_nchw_v1":"yolov5_rk_sigmoid_nchw_v1"},{"anchors",anchors},
      {"input",QJsonObject{{"width",640},{"height",640},{"layout","NHWC"},{"format","RGB8"},
        {"mean",QJsonArray{0,0,0}},{"std",QJsonArray{255,255,255}}}}};
  }
  static void send(AdasController *c,VisionFrameResult p) { c->detector_->result_ready(std::move(p));c->tick(); }
  static void send(AdasController *c,adas::GeometryObservation p) { c->geometry_->result_ready(std::move(p));c->tick(); }
  static void seed(AdasController *c,uint64_t first=1,adas::LineType type=adas::LineType::Dashed) {
    for(uint64_t sequence=first;sequence<first+3;++sequence) {send(c,scene(c,sequence));send(c,road(c,sequence,type));}
  }
  static QJsonObject report(AdasController *c) {
    QTemporaryDir dir;c->write_report(dir.filePath("report.json"));QFile file(dir.filePath("report.json"));
    if(!file.open(QIODevice::ReadOnly))return {};
    return QJsonDocument::fromJson(file.readAll()).object();
  }
private slots:
  void verified_schema3_manifest_selects_profile_and_hash_failure_denies_it() {
    QTemporaryDir dir;QVERIFY(dir.isValid());
    const auto traffic=model_fixture(dir,"traffic.rknn",true),vehicle=model_fixture(dir,"vehicle.rknn",false);
    QVERIFY(!traffic.isEmpty()&&!vehicle.isEmpty());
    const QJsonObject manifest{{"schema",3},{"target","rk3568"},{"version","ui-dual-test"},{"toolkit_version","2.3.2"},
      {"traffic",traffic},{"vehicle",vehicle}};
    QFile file(dir.filePath("manifest.json"));QVERIFY(file.open(QIODevice::WriteOnly));
    QVERIFY(file.write(QJsonDocument(manifest).toJson())>0);file.close();
    const auto saved=qgetenv("ADAS_MODEL_MANIFEST");qputenv("ADAS_MODEL_MANIFEST",file.fileName().toUtf8());
    {
      // No source frame is submitted: fixture model bytes are only hashed,
      // never passed to the RGA/NPU runtime.
      MainWindow window;auto *c=window.findChild<AdasController*>();QVERIFY(c);c->timer_.stop();
      QCOMPARE(c->engine_.required_obstacle_components,adas::ObjectObservation::Combined);
      QVERIFY(c->engine_.separate_crosswalk_source);c->advance_generation();
      QCOMPARE(c->engine_.required_obstacle_components,adas::ObjectObservation::Combined);
      QVERIFY(c->engine_.separate_crosswalk_source);QVERIFY(window.close());
    }
    QFile changed(dir.filePath("vehicle.rknn"));QVERIFY(changed.open(QIODevice::WriteOnly|QIODevice::Append));
    QVERIFY(changed.write("changed")>0);changed.close();
    {
      MainWindow window;auto *c=window.findChild<AdasController*>();QVERIFY(c);c->timer_.stop();
      QCOMPARE(c->engine_.required_obstacle_components,adas::ObjectObservation::All);
      QVERIFY(!c->engine_.separate_crosswalk_source);QVERIFY(window.close());
    }
    if(saved.isNull())qunsetenv("ADAS_MODEL_MANIFEST");else qputenv("ADAS_MODEL_MANIFEST",saved);
  }
  void crosswalk_latest_mailbox_is_independent_of_primary() {
    MainWindow window;auto *c=control(window);QVERIFY(c);c->timer_.stop();dual(c);
    auto p=primary(c,100);p.observation.obstacles={{1,7,{20,20,80,100},.9f,-1}};
    c->queue_result(p);
    for(uint64_t sequence=1;sequence<=50;++sequence)c->queue_result(crossing(c,sequence));
    c->queue_result(crossing(c,49,true)); // Reverse arrival cannot replace the newest traffic result.
    c->tick();
    QCOMPARE(c->views_[0].stamp.sequence,uint64_t(100));
    QCOMPARE(c->vehicle_views_[0].stamp.sequence,uint64_t(100));
    QCOMPARE(c->views_[0].observation.obstacles.size(),size_t(1));
    QCOMPARE(report(c)["crosswalk_sequence"].toDouble(),50.);
    QCOMPARE(c->detection_count_[0],uint64_t(1));
    QVERIFY(c->result_overwritten_.load()>=49);
    const auto people_stamp=c->views_[0].stamp.receive_s;
    send(c,crossing(c,51));
    QCOMPARE(report(c)["crosswalk_sequence"].toDouble(),51.);
    QCOMPARE(c->views_[0].stamp.receive_s,people_stamp);
    QCOMPARE(c->views_[0].stamp.sequence,uint64_t(100));
    QVERIFY(window.close());
  }
  void crosswalk_ui_votes_are_source_specific_and_deduplicated() {
    MainWindow window;auto *c=control(window);QVERIFY(c);c->timer_.stop();dual(c);
    send(c,primary(c,1));send(c,crossing(c,1,true));
    auto second=crossing(c,2,true);send(c,second);
    for(int i=0;i<10;++i){send(c,second);c->tick();}
    QVERIFY(!c->decision_.brake_now);
    send(c,crossing(c,3,true));QVERIFY(c->decision_.brake_now);
    for(uint64_t sequence=2;sequence<=6;++sequence)send(c,primary(c,sequence));
    QVERIFY(c->decision_.brake_now);
    send(c,crossing(c,4));send(c,crossing(c,5));QVERIFY(c->decision_.brake_now);
    send(c,crossing(c,4));QVERIFY(c->decision_.brake_now);
    send(c,crossing(c,6));QVERIFY(!c->decision_.brake_now);
    auto old=crossing(c,100,true);const auto previous=c->generation_.load();
    c->reset_decision();QVERIFY(c->generation_.load()>previous);send(c,old);
    QCOMPARE(report(c)["crosswalk_sequence"].toDouble(),0.);
    QVERIFY(!c->views_[0].stamp.sequence&&!c->vehicle_views_[0].stamp.sequence);
    QCOMPARE(c->engine_.required_obstacle_components,adas::ObjectObservation::Combined);
    QVERIFY(c->engine_.separate_crosswalk_source&&!c->decision_.brake_now);
    QVERIFY(window.close());
  }
  void crosswalk_overlay_has_independent_freshness_and_component_filter() {
    MainWindow window;window.show();window.set_single_view(true);auto *c=control(window);QVERIFY(c);c->timer_.stop();dual(c);
    std::vector<unsigned char> pixels(1280*720*2,0x32);const double now=adas::monotonic_seconds();
    auto frame=c->live_pool_.copy(pixels.data(),1280,720,2560,{c->generation_.load(),1,now,now,false},adas::Layout::Single);
    QVERIFY(frame);c->display_mailbox_.publish(frame);c->tick();send(c,primary(c,100));
    auto x=crossing(c,1);x.stamp.receive_s=x.stamp.source_s=now;x.observation.stamp=x.stamp;
    OverlayBoxResult aux;aux.left=100;aux.top=100;aux.right=200;aux.bottom=200;
    aux.class_id=12;aux.color_r=255;aux.color_g=0;aux.color_b=255;aux.label="aux-person";
    auto vehicle=aux;vehicle.class_id=10;vehicle.left=300;vehicle.right=400;vehicle.label="wrong-vehicle";
    auto block=aux;block.class_id=11;block.left=500;block.right=600;block.label="wrong-roadblock";
    x.overlays={aux,vehicle,block};send(c,x);
    c->render(now);auto image=c->rendered_.toImage();
    const QColor visible(image.pixel(100,150));QVERIFY(visible.red()>200&&visible.blue()>200&&visible.green()<80);
    const QColor excluded(image.pixel(300,150));QVERIFY(!(excluded.red()>200&&excluded.blue()>200&&excluded.green()<80));
    const QColor excluded_block(image.pixel(500,150));QVERIFY(!(excluded_block.red()>200&&excluded_block.blue()>200&&excluded_block.green()<80));
    c->render(now+.501);image=c->rendered_.toImage();
    const QColor expired(image.pixel(100,150));QVERIFY(!(expired.red()>200&&expired.blue()>200&&expired.green()<80));
    QCOMPARE(c->views_[0].overlays.size(),size_t(0));
    QVERIFY(window.close());
  }
  void dual_profile_report_has_independent_crosswalk_status_and_age() {
    MainWindow window;auto *c=control(window);QVERIFY(c);c->timer_.stop();dual(c);
    send(c,primary(c,5));send(c,crossing(c,9));
    const auto r=report(c);
    QVERIFY(r["dual_model_profile"].toBool());
    QCOMPARE(r["required_obstacle_components"].toInt(),int(adas::ObjectObservation::Combined));
    QVERIFY(r["crosswalk_ready"].toBool());QCOMPARE(r["crosswalk_sequence"].toDouble(),9.);
    QVERIFY(r["crosswalk_age_ms"].isArray());
    QCOMPARE(r["crosswalk_age_ms"].toArray().size(),4);
    QVERIFY(r["crosswalk_age_ms"].toArray()[0].toDouble()>=0);
    QVERIFY(!r["roadblock_ready"].toBool());
    QVERIFY(c->capability_label_->text().contains(QStringLiteral("斑马线")));
    QVERIFY(c->capability_label_->text().contains(QStringLiteral("路障识别：未配置")));
    c->advance_generation();
    const auto reset=report(c);QVERIFY(!reset["crosswalk_ready"].toBool());
    QCOMPARE(reset["crosswalk_sequence"].toDouble(),0.);
    QCOMPARE(reset["crosswalk_age_ms"].toArray()[0].toDouble(),-1.);
    QVERIFY(reset["dual_model_profile"].toBool());QVERIFY(window.close());
  }
  void invalid_manifest_and_user_timing_cannot_select_a_lighter_profile() {
    const auto saved=qgetenv("ADAS_MODEL_MANIFEST");
    qputenv("ADAS_MODEL_MANIFEST","/nonexistent/unverified-schema3-manifest.json");
    MainWindow window;auto *c=window.findChild<AdasController*>();QVERIFY(c);c->timer_.stop();
    QCOMPARE(c->engine_.required_obstacle_components,adas::ObjectObservation::All);
    QVERIFY(!c->engine_.separate_crosswalk_source);
    QTemporaryDir dir;QSettings settings(dir.filePath("capabilities.ini"),QSettings::IniFormat);
    settings.setValue("required_obstacle_components",adas::ObjectObservation::Combined);
    settings.setValue("separate_crosswalk_source",true);
    settings.setValue("timing/required_obstacle_components",adas::ObjectObservation::Combined);
    c->load_timing(settings);
    QCOMPARE(c->engine_.required_obstacle_components,adas::ObjectObservation::All);
    QVERIFY(!c->engine_.separate_crosswalk_source);QVERIFY(window.close());
    if(saved.isNull())qunsetenv("ADAS_MODEL_MANIFEST");else qputenv("ADAS_MODEL_MANIFEST",saved);
  }
  void fpga_input_controls_without_arm_file_or_processing() {
    MainWindow window;window.show();
    QVERIFY(!window.findChild<QPushButton*>("adasOpenFile"));
    QVERIFY(!window.findChild<QCheckBox*>("adasPause"));
    QVERIFY(!window.findChild<QCheckBox*>("adasLoop"));
    QVERIFY(!window.findChild<QCheckBox*>("adasVisualSpeed"));
    QVERIFY(!window.findChild<QComboBox*>("adasSpeedSource"));
    QVERIFY(!window.findChild<QPushButton*>("pushButton"));
    QVERIFY(!window.findChild<QPushButton*>("Gray"));
    QVERIFY(!window.findChild<QPushButton*>("save_video"));
    QCOMPARE(window.metaObject()->indexOfSlot("on_pushButton_clicked()"),-1);
    QCOMPARE(window.metaObject()->indexOfSlot("on_Gray_clicked()"),-1);
    for(auto *combo:window.findChildren<QComboBox*>())
      for(int i=0;i<combo->count();++i)QVERIFY(!combo->itemText(i).contains(QStringLiteral("ARM ")));
    for(const char *name:{"Key1","connect","cam1","cam2","save","dma_write","dma_read",
        "radioCamera","radioHDMI","chkPreprocess","chkPlateOverlay","chkWhiteBalance","chkDehaze"})
      QVERIFY(window.findChild<QAbstractButton*>(name));
    for(auto *object:window.findChildren<QObject*>()) {
      const QString type=QString::fromLatin1(object->metaObject()->className());
      QVERIFY(!type.contains("FileVideoSource"));
    }
    QVERIFY(window.close());
  }
  void only_adas_and_raw_modes_reset_generation_and_safety() {
    MainWindow window;window.show();window.set_single_view(true);
    auto *c=control(window);QVERIFY(c);c->timer_.stop();dual(c);
    auto *adas_button=window.findChild<QPushButton*>("adasMode");
    auto *raw_button=window.findChild<QPushButton*>("cam1");QVERIFY(adas_button&&raw_button);
    QVERIFY(adas_button->isChecked()&&!raw_button->isChecked());
    const auto initial_generation=c->generation_.load();
    window.set_detection_mode(static_cast<DetectionMode>(0));window.set_detection_mode(static_cast<DetectionMode>(1));
    QCOMPARE(int(c->mode_),int(DetectionMode::Adas));QCOMPARE(c->generation_.load(),initial_generation);
    send(c,primary(c,1));send(c,crossing(c,1,true));send(c,crossing(c,2,true));send(c,crossing(c,3,true));
    QVERIFY(c->decision_.brake_now);
    const auto old_scene=primary(c,100),old_crosswalk=crossing(c,100,true);
    const auto old_lane=road(c,100);const auto first=c->generation_.load();
    QTest::mouseClick(raw_button,Qt::LeftButton);c->tick();
    QCOMPARE(int(c->mode_),int(DetectionMode::RawVideo));QVERIFY(c->generation_.load()>first);
    QVERIFY(!adas_button->isChecked()&&raw_button->isChecked());
    QVERIFY(!c->views_[0].stamp.sequence&&!c->crosswalk_views_[0].stamp.sequence&&!c->last_geometry_.stamp.sequence);
    send(c,old_scene);send(c,old_crosswalk);send(c,old_lane);
    QVERIFY(!c->views_[0].stamp.sequence&&!c->crosswalk_views_[0].stamp.sequence&&!c->last_geometry_.stamp.sequence);
    const auto raw_generation=c->generation_.load();
    QTest::mouseClick(adas_button,Qt::LeftButton);c->tick();
    QCOMPARE(int(c->mode_),int(DetectionMode::Adas));QVERIFY(c->generation_.load()>raw_generation);
    QVERIFY(adas_button->isChecked()&&!raw_button->isChecked());
    QVERIFY(c->decision_.stop_required&&!c->decision_.auto_driving&&!c->decision_.lane_change_allowed);
    QCOMPARE(c->engine_.required_obstacle_components,adas::ObjectObservation::Combined);
    QVERIFY(c->engine_.separate_crosswalk_source);
    send(c,old_scene);send(c,old_crosswalk);send(c,old_lane);
    QVERIFY(!c->views_[0].stamp.sequence&&!c->crosswalk_views_[0].stamp.sequence&&!c->last_geometry_.stamp.sequence);
    QVERIFY(c->decision_.stop_required&&!c->decision_.lane_change_allowed);
    send(c,primary(c,1));send(c,crossing(c,1,true));send(c,crossing(c,2,true));
    QVERIFY(!c->decision_.brake_now);send(c,crossing(c,3,true));QVERIFY(c->decision_.brake_now);
    QVERIFY(window.close());
  }
  void keyboard_and_simulation_speed_controls() {
    MainWindow window;window.resize(1024,600);window.show();auto *c=control(window);QVERIFY(c);c->timer_.stop();c->tick();
    auto *lane=window.findChild<QComboBox*>("adasLane");auto *speed=window.findChild<QDoubleSpinBox*>("adasManualSpeed");
    QVERIFY(lane&&speed);
    QCOMPARE(speed->value(),0.);QVERIFY(speed->isEnabled());QVERIFY(c->decision_.speed_valid);
    QCOMPARE(int(c->decision_.speed_source),int(adas::SpeedSource::Simulation));
    window.setFocus();QTest::keyClick(&window,Qt::Key_1);QCOMPARE(lane->currentIndex(),0);
    for(int index:{1,2,3,0}){QTest::keyClick(&window,Qt::Key_N);QCOMPARE(c->nav_combo_->currentIndex(),index);}
    QVERIFY(!c->ego_.manual_nav_enabled);
    speed->setFocus();QTest::keyClick(speed,Qt::Key_3);QCOMPARE(lane->currentIndex(),0);
    speed->clearFocus();window.setFocus();QTest::keyClick(&window,Qt::Key_Space);
    QVERIFY(window.findChild<QCheckBox*>("adasManualStop")->isChecked());
    speed->setValue(80);c->tick();QVERIFY(c->decision_.alarm_overspeed);
    speed->setValue(0);c->tick();QVERIFY(speed->isEnabled());
    QVERIFY(c->decision_.speed_valid&&!c->decision_.alarm_overspeed);
    QVERIFY(window.size().width()<=1024&&window.size().height()<=600);QVERIFY(window.close());
  }
  void arrows_require_explicit_force_and_refresh_same_direction() {
    MainWindow window;window.show();auto *c=control(window);QVERIFY(c);c->timer_.stop();window.setFocus();
    QTest::keyClick(&window,Qt::Key_Up,Qt::ControlModifier);QVERIFY(c->ego_.force_drive&&!c->ego_.manual_stop);
    c->auto_check_->setChecked(false);QTest::keyClick(&window,Qt::Key_Up);QVERIFY(!c->ego_.force_drive&&!c->ego_.manual_stop&&c->ego_.automatic);
    QTest::keyClick(&window,Qt::Key_Right,Qt::ControlModifier);QCOMPARE(int(c->ego_.force_lane_change),int(adas::TurnSignal::Right));
    QTest::keyClick(&window,Qt::Key_Left);QCOMPARE(int(c->ego_.force_lane_change),int(adas::TurnSignal::None));
    QCOMPARE(int(c->ego_.manual_nav),int(adas::Direction::Left));QVERIFY(c->ego_.manual_nav_enabled);
    const double request=c->ego_.lane_change_request_s;QVERIFY(request>0);
    QTest::qWait(2);QTest::keyClick(&window,Qt::Key_Left);QVERIFY(c->ego_.lane_change_request_s>request);
    QTest::keyClick(&window,Qt::Key_Down);QVERIFY(c->ego_.manual_stop&&!c->ego_.force_drive);
    QTest::keyClick(&window,Qt::Key_Right);QVERIFY(c->ego_.manual_stop); // Navigation never releases parking.
    QTest::keyClick(&window,Qt::Key_Escape);QVERIFY(!c->ego_.manual_nav_enabled&&!c->ego_.force_drive);
    auto *editor=window.findChild<QDoubleSpinBox*>("adasManualSpeed");editor->setFocus();
    QTest::keyClick(editor,Qt::Key_Up,Qt::ControlModifier);QVERIFY(!c->ego_.force_drive);
    QVERIFY(window.close());
  }
  void navigation_combo_cancels_previous_forced_action_and_turn() {
    MainWindow window;window.show();auto *c=control(window);QVERIFY(c);c->timer_.stop();window.setFocus();
    QTest::keyClick(&window,Qt::Key_Left);
    QTest::keyClick(&window,Qt::Key_Right,Qt::ControlModifier);
    QVERIFY(c->ego_.force_lane_change==adas::TurnSignal::Right);
    c->manual_stop_check_->setChecked(true);
    c->nav_combo_->setCurrentIndex(int(adas::Direction::Straight));
    QCOMPARE(int(c->ego_.force_lane_change),int(adas::TurnSignal::None));
    QCOMPARE(int(c->ego_.signal),int(adas::TurnSignal::None));
    QVERIFY(c->ego_.manual_stop&&!c->ego_.force_drive);
    QCOMPARE(int(c->ego_.manual_nav),int(adas::Direction::Straight));
    QCOMPARE(c->ego_.lane_change_request_s,0.);
    c->force_drive_check_->setChecked(true);
    c->nav_combo_->setCurrentIndex(int(adas::Direction::Right));
    QVERIFY(!c->ego_.force_drive&&c->ego_.manual_stop);
    QCOMPARE(int(c->ego_.signal),int(adas::TurnSignal::Right));
    c->force_lane_combo_->setCurrentIndex(int(adas::TurnSignal::Left));
    c->nav_combo_->setCurrentIndex(0);
    QVERIFY(!c->ego_.manual_nav_enabled&&!c->ego_.force_drive&&c->ego_.manual_stop);
    QCOMPARE(int(c->ego_.force_lane_change),int(adas::TurnSignal::None));
    QCOMPARE(int(c->ego_.signal),int(adas::TurnSignal::None));
    QVERIFY(window.close());
  }
  void reselecting_navigation_cancels_force_and_refreshes_request() {
    MainWindow window;window.show();auto *c=control(window);QVERIFY(c);c->timer_.stop();window.setFocus();
    QTest::keyClick(&window,Qt::Key_Left);
    QTest::keyClick(&window,Qt::Key_Right,Qt::ControlModifier);
    QCOMPARE(c->nav_combo_->currentIndex(),int(adas::Direction::Left));
    QVERIFY(QMetaObject::invokeMethod(c->nav_combo_,"activated",Q_ARG(int,int(adas::Direction::Left))));
    QCOMPARE(int(c->ego_.force_lane_change),int(adas::TurnSignal::None));
    QCOMPARE(int(c->ego_.signal),int(adas::TurnSignal::Left));
    QTest::keyClick(&window,Qt::Key_Up,Qt::ControlModifier);
    c->manual_stop_check_->setChecked(true);
    QVERIFY(QMetaObject::invokeMethod(c->nav_combo_,"activated",Q_ARG(int,int(adas::Direction::Left))));
    QVERIFY(!c->ego_.force_drive&&c->ego_.manual_stop);
    const double request=c->ego_.lane_change_request_s;QTest::qWait(2);
    QVERIFY(QMetaObject::invokeMethod(c->nav_combo_,"activated",Q_ARG(int,int(adas::Direction::Left))));
    QVERIFY(c->ego_.lane_change_request_s>request);
    QVERIFY(window.close());
  }
  void cached_prediction_overlay_expires_before_detection_ttl() {
    MainWindow window;window.show();window.set_single_view(true);auto *c=control(window);QVERIFY(c);c->timer_.stop();
    std::vector<unsigned char> pixels(1280*720*2,0x32);const double now=adas::monotonic_seconds();
    auto frame=c->live_pool_.copy(pixels.data(),1280,720,2560,{c->generation_.load(),1,now,now,false},adas::Layout::Single);
    c->display_mailbox_.publish(frame);c->tick();
    auto r=scene(c,1);r.stamp.receive_s=r.stamp.source_s=now;r.observation.stamp=r.stamp;
    OverlayBoxResult b;b.left=100;b.top=100;b.right=200;b.bottom=200;b.class_id=1;b.color_r=255;b.color_g=190;b.color_b=50;
    b.predicted=true;b.last_observed_s=now-.1;b.track_hits=2;b.track_id=42;b.label="person";r.overlays={b};send(c,r);
    c->render(now);const auto before=c->rendered_.toImage().copy(95,95,115,115);
    c->render(now+.26);const auto after=c->rendered_.toImage().copy(95,95,115,115);
    QVERIFY(before!=after); // Whole observation is fresh, but predicted box is expired.
    QVERIFY(window.close());
  }
  void current_person_report_excludes_expired_measurements() {
    MainWindow window;auto *c=control(window);QVERIFY(c);c->timer_.stop();
    auto r=scene(c,1);r.observation.obstacles={{1,42,{10,10,40,80},.9f,-1}};send(c,r);
    QCOMPARE(report(c)["front_and_side_person_measured"].toInt(),1);
    c->views_[0].stamp.receive_s-=2;c->views_[0].observation.stamp=c->views_[0].stamp;
    QCOMPARE(report(c)["front_and_side_person_measured"].toInt(),0);QVERIFY(window.close());
  }
  void cpu_lane_runs_without_a_neural_lane_model() {
    MainWindow window;window.show();window.set_single_view(true);auto *c=control(window);QVERIFY(c);c->timer_.stop();
    QCOMPARE(window.findChildren<adas::GeometryWorker*>().size(),1);
    std::vector<unsigned char> pixels(1280*720*2,0);const double now=adas::monotonic_seconds();
    auto frame=c->live_pool_.copy(pixels.data(),1280,720,2560,{c->generation_.load(),1,now,now,false},adas::Layout::Single);
    c->display_mailbox_.publish(frame);c->geometry_->submit(frame);
    QTRY_VERIFY((c->tick(),c->last_geometry_.stamp.sequence==1));
    QCOMPARE(int(c->last_geometry_.status),int(adas::Status::Valid));
    QCOMPARE(c->last_geometry_.components,adas::GeometryObservation::Lane);
    const auto r=report(c);QCOMPARE(r["lane_backend"].toString(),QStringLiteral("classical_cpu"));
    QVERIFY(r["lane_cpu_hz_average"].toDouble()>0);QCOMPARE(r["lane_npu_hz_average"].toDouble(),0.);
    QVERIFY(window.close());
  }
  void static_signals_manual_force_and_visible_braking() {
    MainWindow window;window.show();window.set_single_view(true);auto *c=control(window);QVERIFY(c);c->timer_.stop();window.setFocus();
    std::vector<unsigned char> pixels(1280*720*2,0x32);const double now=adas::monotonic_seconds();
    auto frame=c->live_pool_.copy(pixels.data(),1280,720,2560,{c->generation_.load(),1,now,now,true},adas::Layout::Single);
    c->display_mailbox_.publish(frame);c->tick();
    auto red=scene(c,1);red.stamp.static_image=true;red.observation.stamp=red.stamp;red.observation.light=adas::LightState::Red;
    send(c,red);QVERIFY(c->decision_.stop_required&&!c->decision_.auto_driving);
    QTest::keyClick(&window,Qt::Key_1);QTest::keyClick(&window,Qt::Key_Up,Qt::ControlModifier);c->tick();
    QVERIFY(c->decision_.alarm_red_run&&c->decision_.auto_driving);QCOMPARE(int(c->decision_.arrow),int(adas::Direction::Left));
    QTest::keyClick(&window,Qt::Key_Up);c->tick();QVERIFY(!c->decision_.auto_driving&&!c->decision_.alarm_red_run);
    QTest::keyClick(&window,Qt::Key_Down);auto green=scene(c,2);green.stamp.static_image=true;green.observation.stamp=green.stamp;green.observation.light=adas::LightState::Green;send(c,green);
    QVERIFY(!c->decision_.auto_driving&&c->ego_.manual_stop);
    QTest::keyClick(&window,Qt::Key_Up);c->tick();QVERIFY(c->decision_.auto_driving);
    auto hazard=scene(c,3);hazard.stamp.static_image=true;hazard.observation.stamp=hazard.stamp;hazard.observation.light=adas::LightState::Green;
    hazard.observation.obstacles={{1,42,{560,300,700,680},.9f,-1}};send(c,hazard);
    QVERIFY(c->decision_.brake_now&&!c->decision_.auto_driving);
    const QColor fill(c->rendered_.toImage().pixel(1280/8+5,720/4+5));QVERIFY(fill.red()>fill.green()+50);
    QTest::keyClick(&window,Qt::Key_Up,Qt::ControlModifier);c->tick();QVERIFY(c->decision_.brake_now&&!c->decision_.auto_driving);
    const auto evidence=qEnvironmentVariable("ADAS_TEST_EVIDENCE_DIR");if(!evidence.isEmpty())c->screenshot(evidence+"/ui-brake-warning.png");
    QVERIFY(window.close());
  }
  void single_input_uses_complete_front_capabilities_and_path() {
    MainWindow window;window.show();window.set_single_view(true);auto *c=control(window);QVERIFY(c);c->timer_.stop();
    seed(c);window.findChild<QComboBox*>("adasTurnSignal")->setCurrentIndex(2);c->tick();
    QVERIFY(c->decision_.lane_change_allowed);QCOMPARE(c->decision_.local_plan.point_count,16u);
    QCOMPARE(int(c->decision_.local_plan.maneuver),int(adas::Maneuver::ChangeRight));
    auto p=scene(c,4);p.observation.roadblock_status=adas::Status::Missing;send(c,p);
    QVERIFY(!c->decision_.lane_change_allowed&&c->decision_.stop_required);
    p=scene(c,5);p.observation.obstacles={{10,1,{1100,440,1250,660},.9f,-1}};send(c,p);
    QVERIFY(!c->decision_.lane_change_allowed); // Visible target lane obstacle.
    seed(c,6,adas::LineType::Solid);window.findChild<QComboBox*>("adasForceLane")->setCurrentIndex(2);c->tick();
    QVERIFY(c->decision_.alarm_illegal_lane_change&&!c->decision_.lane_change_allowed);
    QVERIFY(window.close());
  }
  void four_view_requires_new_target_side_and_zoom_preserves_gate() {
    MainWindow window;window.show();auto *c=control(window);QVERIFY(c);c->timer_.stop();seed(c);
    auto old_side=scene(c,10,adas::ViewId::Right);
    window.findChild<QComboBox*>("adasTurnSignal")->setCurrentIndex(2);
    old_side.stamp.receive_s=old_side.stamp.source_s=c->ego_.lane_change_request_s-.01;old_side.observation.stamp=old_side.stamp;
    send(c,old_side);QVERIFY(!c->decision_.lane_change_allowed);
    send(c,scene(c,11,adas::ViewId::Right));QVERIFY(c->decision_.lane_change_allowed);
    auto missing=scene(c,12,adas::ViewId::Right);missing.observation.roadblock_status=adas::Status::Missing;send(c,missing);
    QVERIFY(!c->decision_.lane_change_allowed);
    window.findChild<QCheckBox*>("adasFrontView")->setChecked(true);c->tick();
    QCOMPARE(int(c->ego_.input_layout),int(adas::Layout::Mosaic));QVERIFY(!c->decision_.lane_change_allowed);
    // An actual camera source cannot be overridden into single-input authorization.
    c->live_source_known_=true;window.set_single_view(true);
    QCOMPARE(int(c->ego_.input_layout),int(adas::Layout::Mosaic));
    QVERIFY(window.close());
  }
  void roi_signal_isolated_from_objects_and_braking() {
    MainWindow window;window.show();auto *c=control(window);QVERIFY(c);c->timer_.stop();seed(c);
    auto hazard=scene(c,10);hazard.observation.obstacles={{1,1,{280,180,360,350},.95f,-1}};send(c,hazard);
    QVERIFY(c->decision_.brake_now);const auto captured=c->views_[0].stamp.sequence;
    for(uint64_t sequence=11;sequence<=13;++sequence) {
      auto roi=scene(c,sequence);roi.observation.components=adas::ObjectObservation::Signal;
      roi.observation.light=adas::LightState::Green;send(c,roi);
    }
    QCOMPARE(c->views_[0].stamp.sequence,captured);QCOMPARE(c->vehicle_views_[0].stamp.sequence,captured);
    QVERIFY(c->decision_.brake_now);QCOMPARE(int(c->decision_.light),int(adas::LightState::Green));
    QCOMPARE(c->views_[0].observation.obstacles.size(),size_t(1));QVERIFY(window.close());
  }
  void lane_navigation_ordering_and_old_source_rejection() {
    MainWindow window;window.show();window.set_single_view(true);auto *c=control(window);QVERIFY(c);c->timer_.stop();seed(c,100);
    for(uint64_t seq=20;seq<=22;++seq) {
      auto nav=road(c,seq);nav.components=adas::GeometryObservation::Navigation;nav.nav={true,adas::Direction::Right,.9f,{}};send(c,nav);
    }
    QCOMPARE(c->last_geometry_.stamp.sequence,uint64_t(102));
    QVERIFY(!c->ego_.manual_nav_enabled);QCOMPARE(c->ego_.lane_change_request_s,0.);
    QCOMPARE(int(c->decision_.arrow),int(adas::Direction::Straight));
    c->nav_combo_->setCurrentIndex(3);c->tick();
    QVERIFY(c->decision_.lane_change_allowed);QCOMPARE(int(c->decision_.arrow),int(adas::Direction::Right));
    const double requested=c->ego_.lane_change_request_s;
    auto stale=road(c,21,adas::LineType::Solid);stale.components=adas::GeometryObservation::Navigation;stale.status=adas::Status::Failed;send(c,stale);
    QCOMPARE(int(c->ego_.manual_nav),int(adas::Direction::Right));QCOMPARE(c->ego_.lane_change_request_s,requested);
    auto failure=road(c,103);failure.status=adas::Status::Failed;send(c,failure);
    QCOMPARE(int(c->ego_.manual_nav),int(adas::Direction::Right));QCOMPARE(c->ego_.lane_change_request_s,requested);QVERIFY(!c->decision_.lane_change_allowed);
    c->nav_combo_->setCurrentIndex(0);c->tick();
    QCOMPARE(c->ego_.lane_change_request_s,0.);QVERIFY(!c->ego_.manual_nav_enabled);
    auto previous_scene=scene(c,1000);auto previous_lane=road(c,1000);const auto epoch=c->generation_.load();
    c->reset_decision();QVERIFY(c->generation_.load()>epoch);send(c,previous_scene);send(c,previous_lane);
    QVERIFY(!c->views_[0].stamp.sequence&&!c->last_geometry_.stamp.sequence&&c->decision_.stop_required);
    QVERIFY(window.close());
  }
  void missing_lane_is_not_counted_as_npu_and_denies_change() {
    MainWindow window;window.show();window.set_single_view(true);auto *c=control(window);QVERIFY(c);c->timer_.stop();
    std::vector<unsigned char> pixels(1280*720*2,0x32);const double now=adas::monotonic_seconds();
    auto frame=c->live_pool_.copy(pixels.data(),1280,720,2560,{c->generation_.load(),1,now,now,false},adas::Layout::Single);
    QVERIFY(frame);c->display_mailbox_.publish(frame);c->tick();QVERIFY(!c->rendered_.isNull());
    for(uint64_t seq=1;seq<=3;++seq){send(c,scene(c,seq));auto lane=road(c,seq);lane.status=adas::Status::Missing;send(c,lane);}
    c->nav_combo_->setCurrentIndex(3);c->tick();
    QCOMPARE(c->geometry_count_,uint64_t(0));QVERIFY(!c->decision_.lane_change_allowed);
    const auto result=report(c);QCOMPARE(result["lane_npu_hz_average"].toDouble(),0.);
    QCOMPARE(result["geometry_receive_to_hud_p95_ms"].toDouble(),-1.);QVERIFY(window.close());
  }
  void bounded_mailboxes_keep_latest_without_repeated_votes() {
    MainWindow window;auto *c=control(window);QVERIFY(c);c->timer_.stop();
    for(uint64_t seq=1;seq<=1000;++seq)c->detector_->result_ready(scene(c,seq));
    for(int kind=0;kind<3;++kind)QVERIFY(c->pending_result_ready_[0][kind]);
    QVERIFY(c->result_overwritten_.load()>=2997);QCOMPARE(c->detection_count_[0],uint64_t(0));
    c->tick();QCOMPARE(c->detection_count_[0],uint64_t(1));QCOMPARE(c->views_[0].stamp.sequence,uint64_t(1000));
    auto nav=road(c,1002);nav.components=adas::GeometryObservation::Navigation;nav.nav={true,adas::Direction::Left,.9f,{}};
    c->queue_geometry(road(c,1001));c->queue_geometry(nav);c->tick();
    QCOMPARE(c->last_geometry_.stamp.sequence,uint64_t(1001));
    for(int i=0;i<10;++i){send(c,nav);c->tick();}
    QCOMPARE(int(c->decision_.arrow),int(adas::Direction::Straight)); // A repeated source frame is one confirmation.
    QVERIFY(window.close());
  }
  void reset_discards_preview_without_restamping_or_resubmitting() {
    MainWindow window;window.show();auto *c=control(window);QVERIFY(c);c->timer_.stop();
    std::vector<unsigned char> pixels(1280*720*2,0x32);const double now=adas::monotonic_seconds();
    auto frame=c->live_pool_.copy(pixels.data(),1280,720,2560,{c->generation_.load(),1,now,now,false},adas::Layout::Mosaic);
    QVERIFY(frame);c->display_mailbox_.publish(frame);c->tick();QVERIFY(c->displayed_);QVERIFY(!c->rendered_.isNull());
    const auto old_epoch=c->generation_.load();const auto old_captured=c->captured_.load();
    c->reset_decision();c->tick();QVERIFY(c->generation_.load()>old_epoch);
    QVERIFY(!c->displayed_&&c->rendered_.isNull());QCOMPARE(c->captured_.load(),old_captured);
    QCOMPARE(report(c)["input"].toString(),QStringLiteral("fpga_pcie"));QVERIFY(window.close());
  }
  void unconfirmed_live_source_cannot_authorize_with_old_layout() {
    MainWindow window;auto *c=control(window);QVERIFY(c);c->timer_.stop();
    c->accepting_live_=true;c->live_source_known_=false;seed(c);
    QVERIFY(c->decision_.stop_required&&!c->decision_.auto_driving);
    QVERIFY(!c->ego_.manual_stop); // Temporary source guard never changes the operator control.
    QVERIFY(window.close());
  }
  void timing_config_cannot_extend_safety_freshness() {
    MainWindow window;auto *c=control(window);QVERIFY(c);c->timer_.stop();QTemporaryDir dir;
    QSettings settings(dir.filePath("timing.ini"),QSettings::IniFormat);
    settings.setValue("timing/front_timeout_s",5.0);settings.setValue("timing/side_timeout_s",10.0);
    c->load_timing(settings);QCOMPARE(c->engine_.front_timeout_s,.5);QCOMPARE(c->engine_.side_timeout_s,1.5);
    settings.setValue("timing/front_timeout_s",std::numeric_limits<double>::quiet_NaN());
    settings.setValue("timing/side_timeout_s",std::numeric_limits<double>::infinity());
    c->load_timing(settings);QCOMPARE(c->engine_.front_timeout_s,.5);QCOMPARE(c->engine_.side_timeout_s,1.5);
    settings.setValue("timing/front_timeout_s",.01);settings.setValue("timing/side_timeout_s",.01);
    c->load_timing(settings);QCOMPARE(c->engine_.front_timeout_s,.1);QCOMPARE(c->engine_.side_timeout_s,.2);
    QVERIFY(window.close());
  }
  void source_switch_invalidates_cached_confirmation_until_status_read() {
    MainWindow window;auto *c=control(window);QVERIFY(c);c->timer_.stop();
    c->live_source_known_=true;seed(c);QVERIFY(!c->decision_.stop_required);
    const auto generation=c->generation_.load();c->physical_source_changed();
    QVERIFY(c->generation_.load()>generation);QVERIFY(!c->live_source_known_);
    QVERIFY(!c->displayed_&&!c->views_[0].stamp.sequence);
    QVERIFY(c->discard_live_before_.load()>adas::monotonic_seconds());
    QVERIFY(window.close());
  }

  void capture_restart_requires_new_source_confirmation() {
    MainWindow window;auto *c=control(window);QVERIFY(c);c->timer_.stop();
    c->live_source_known_=true;seed(c);QVERIFY(!c->decision_.stop_required);
    QVERIFY(c->prepare_live());
    QVERIFY(!c->live_source_known_); // No open device can confirm the previous capture session's layout.
    c->tick();QVERIFY(c->decision_.stop_required);
    QVERIFY(window.close());
  }

};
int main(int argc,char **argv) {
  qputenv("ADAS_MODEL_MANIFEST","/nonexistent/adas-ui-test-manifest.json");
  QApplication app(argc,argv);AdasUiTest test;return QTest::qExec(&test,argc,argv);
}
#include "adas_ui_test.moc"
