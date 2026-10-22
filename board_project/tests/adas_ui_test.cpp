#include "ui/mainwindow.h"
#include "ui/adas_controller.h"
#include "inference/adas_pipeline.h"
#include <QtTest/QtTest>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPushButton>
#include <QTemporaryDir>
#include <limits>

class AdasUiTest : public QObject {
  Q_OBJECT
  static AdasController *control(MainWindow &window) { return window.findChild<AdasController*>(); }
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
  static void send(AdasController *c,VisionFrameResult p) { c->detector_->result_ready(std::move(p));c->tick(); }
  static void send(AdasController *c,adas::GeometryObservation p) { c->detector_->geometry_ready(std::move(p));c->tick(); }
  static void seed(AdasController *c,uint64_t first=1,adas::LineType type=adas::LineType::Dashed) {
    for(uint64_t sequence=first;sequence<first+3;++sequence) {send(c,scene(c,sequence));send(c,road(c,sequence,type));}
  }
  static QJsonObject report(AdasController *c) {
    QTemporaryDir dir;c->write_report(dir.filePath("report.json"));QFile file(dir.filePath("report.json"));
    if(!file.open(QIODevice::ReadOnly))return {};
    return QJsonDocument::fromJson(file.readAll()).object();
  }
private slots:
  void fpga_input_controls_without_arm_file_or_processing() {
    MainWindow window;window.show();
    QVERIFY(!window.findChild<QPushButton*>("adasOpenFile"));
    QVERIFY(!window.findChild<QCheckBox*>("adasPause"));
    QVERIFY(!window.findChild<QCheckBox*>("adasLoop"));
    QVERIFY(!window.findChild<QCheckBox*>("adasVisualSpeed"));
    for(auto *combo:window.findChildren<QComboBox*>())
      for(int i=0;i<combo->count();++i)QVERIFY(!combo->itemText(i).contains(QStringLiteral("ARM ")));
    for(const char *name:{"radioCamera","radioHDMI","chkPreprocess","chkPlateOverlay","chkWhiteBalance","chkDehaze"})
      QVERIFY(window.findChild<QAbstractButton*>(name));
    for(auto *object:window.findChildren<QObject*>()) {
      const QString type=QString::fromLatin1(object->metaObject()->className());
      QVERIFY(!type.contains("FileVideoSource")&&!type.contains("GeometryWorker"));
    }
    QVERIFY(!window.startup_adas_with_file(QStringLiteral("/nonexistent/scene.mp4")));
    QVERIFY(window.close());
  }
  void keyboard_default_speed_and_external_unavailable() {
    MainWindow window;window.resize(1024,600);window.show();auto *c=control(window);QVERIFY(c);c->timer_.stop();c->tick();
    auto *lane=window.findChild<QComboBox*>("adasLane");auto *speed=window.findChild<QDoubleSpinBox*>("adasManualSpeed");
    auto *source=window.findChild<QComboBox*>("adasSpeedSource");QVERIFY(lane&&speed&&source);
    QCOMPARE(speed->value(),0.);QVERIFY(speed->isEnabled());QVERIFY(c->decision_.speed_valid);
    QCOMPARE(int(c->decision_.speed_source),int(adas::SpeedSource::Simulation));
    window.setFocus();QTest::keyClick(&window,Qt::Key_1);QCOMPARE(lane->currentIndex(),0);
    speed->setFocus();QTest::keyClick(speed,Qt::Key_3);QCOMPARE(lane->currentIndex(),0);
    speed->clearFocus();window.setFocus();QTest::keyClick(&window,Qt::Key_Space);
    QVERIFY(window.findChild<QCheckBox*>("adasManualStop")->isChecked());
    speed->setValue(80);c->tick();QVERIFY(c->decision_.alarm_overspeed);
    source->setCurrentIndex(1);c->tick();QVERIFY(!speed->isEnabled());
    QVERIFY(!c->decision_.speed_valid&&!c->decision_.alarm_overspeed);
    QVERIFY(window.size().width()<=1024&&window.size().height()<=600);QVERIFY(window.close());
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
    QCOMPARE(c->last_geometry_.stamp.sequence,uint64_t(102));QCOMPARE(c->last_navigation_.stamp.sequence,uint64_t(22));
    QVERIFY(c->decision_.lane_change_allowed);QCOMPARE(int(c->decision_.arrow),int(adas::Direction::Right));
    auto stale=road(c,21,adas::LineType::Solid);stale.components=adas::GeometryObservation::Navigation;stale.status=adas::Status::Failed;send(c,stale);
    QCOMPARE(c->last_navigation_.stamp.sequence,uint64_t(22));
    auto failure=road(c,103);failure.status=adas::Status::Failed;send(c,failure);
    QCOMPARE(c->last_navigation_.stamp.sequence,uint64_t(22));QVERIFY(!c->decision_.lane_change_allowed);
    auto previous_scene=scene(c,1000);auto previous_lane=road(c,1000);const auto epoch=c->generation_.load();
    c->reset_decision();QVERIFY(c->generation_.load()>epoch);send(c,previous_scene);send(c,previous_lane);
    QVERIFY(!c->views_[0].stamp.sequence&&!c->last_geometry_.stamp.sequence&&c->decision_.stop_required);
    QVERIFY(window.close());
  }
  void bounded_mailboxes_keep_latest_without_repeated_votes() {
    MainWindow window;auto *c=control(window);QVERIFY(c);c->timer_.stop();
    for(uint64_t seq=1;seq<=1000;++seq)c->detector_->result_ready(scene(c,seq));
    for(int kind=0;kind<3;++kind)QVERIFY(c->pending_result_ready_[0][kind]);
    QVERIFY(c->result_overwritten_.load()>=2997);QCOMPARE(c->detection_count_[0],uint64_t(0));
    c->tick();QCOMPARE(c->detection_count_[0],uint64_t(1));QCOMPARE(c->views_[0].stamp.sequence,uint64_t(1000));
    auto nav=road(c,1001);nav.components=adas::GeometryObservation::Navigation;nav.nav={true,adas::Direction::Left,.9f,{}};
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
  qputenv("PANGO_TRAFFIC_DET_MODEL","/nonexistent/adas-ui-test.rknn");
  QApplication app(argc,argv);AdasUiTest test;return QTest::qExec(&test,argc,argv);
}
#include "adas_ui_test.moc"
