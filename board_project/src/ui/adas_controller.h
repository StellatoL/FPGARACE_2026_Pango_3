#ifndef ADAS_CONTROLLER_H
#define ADAS_CONTROLLER_H
#include <QObject>
#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QSettings>
#include <atomic>
#include <mutex>
#include <algorithm>
#include <cmath>
#include "common/frame_packet.h"
#include "common/vision_result.h"
#include "core/adas_engine.h"
#include "core/ground_calibration.h"

class MainWindow;
class pango_fun;
class QCheckBox;
class QComboBox;
class QLabel;
class QDoubleSpinBox;
namespace Ui { class MainWindow; }
namespace adas { class DetectionWorker; class GeometryWorker; enum class ManualAction; }

class AdasController : public QObject {
  Q_OBJECT
public:
  AdasController(MainWindow *window, pango_fun *device, Ui::MainWindow *ui);
  ~AdasController() override;
  void set_mode(DetectionMode mode);
  bool prepare_live();
  bool stop_live();
  bool stop_source();
  bool shutdown();
  void set_layout(adas::Layout layout);
  void write_report(const QString &path) const;
  void screenshot(const QString &path) const;
  void physical_source_changed();
private:
  friend class AdasUiTest;
  void setup_controls();
  void ingest_live(int slot);
  void publish(const adas::FramePacket &frame);
  void advance_generation();
  void queue_result(VisionFrameResult result);
  void queue_geometry(adas::GeometryObservation result);
  void drain_results();
  void accept_result(VisionFrameResult result);
  void accept_geometry(adas::GeometryObservation result);
  void tick();
  void render(double now);
  void refresh_labels(double now);
  void refresh_stats(double now);
  void update_settings();
  void update_ego();
  void apply_manual(adas::ManualAction action);
  void refresh_turn_request(bool new_request = false);
  void calibrate();
  void edit_rois();
  void load_profile();
  void load_timing(QSettings &settings) {
    const double front=settings.value("timing/front_timeout_s",.5).toDouble();
    const double side=settings.value("timing/side_timeout_s",1.5).toDouble();
    engine_.front_timeout_s=std::clamp(std::isfinite(front)?front:.5,.1,.5);
    engine_.side_timeout_s=std::clamp(std::isfinite(side)?side:1.5,.2,1.5);
  }
  void sync_live_source();
  QString profile_key() const;
  bool eventFilter(QObject *object, QEvent *event) override;
  void reset_decision();

  MainWindow *window_;
  pango_fun *device_;
  Ui::MainWindow *ui_;
  adas::FramePool live_pool_;
  adas::LatestFrameMailbox display_mailbox_;
  adas::FramePacket displayed_;
  std::atomic<uint64_t> generation_{1}, sequence_{0};
  std::atomic<double> discard_live_before_{0};
  std::atomic<int> layout_{0};
  std::atomic<bool> accepting_live_{false};
  std::atomic<uint64_t> captured_{0}, pool_dropped_{0};
  uint64_t shown_ = 0, last_captured_ = 0, last_shown_ = 0;
  uint64_t last_presented_sequence_ = 0;
  uint64_t geometry_count_ = 0, previous_geometry_count_ = 0;
  uint64_t last_latency_sequence_ = 0, last_geometry_latency_sequence_ = 0, last_vehicle_latency_sequence_ = 0;
  std::array<uint64_t,4> detection_count_{}, previous_detection_count_{};
  std::array<uint64_t,4> vehicle_count_{}, previous_vehicle_count_{};
  std::array<uint64_t,4> crosswalk_count_{}, previous_crosswalk_count_{};
  std::array<double,4> vehicle_hz_{};
  std::array<double,4> crosswalk_hz_{};
  std::array<double,4> detection_hz_{};
  double preview_hz_ = 0, capture_hz_ = 0, lane_hz_ = 0;
  std::vector<double> detection_latency_, geometry_latency_, vehicle_latency_;
  double started_s_ = 0, last_stats_s_ = 0;
  bool closing_ = false;
  bool live_source_known_ = false;
  DetectionMode mode_ = DetectionMode::Adas;
  adas::DetectionWorker *detector_ = nullptr;
  adas::GeometryWorker *geometry_ = nullptr;
  std::mutex results_mutex_;
  std::array<std::array<VisionFrameResult,5>,4> pending_results_{};
  std::array<std::array<bool,5>,4> pending_result_ready_{};
  adas::GeometryObservation pending_geometry_;
  bool pending_geometry_ready_ = false;
  std::atomic<uint64_t> result_overwritten_{0};
  QTimer timer_;
  adas::AdasEngine engine_;
  adas::EgoState ego_;
  adas::AdasDecision decision_;
  adas::GroundCalibration calibration_;
  std::array<adas::Roi,3> light_rois_{{{0,0,1,.5},{0,0,1,.5},{0,0,1,.5}}};
  std::array<VisionFrameResult,4> views_;
  std::array<VisionFrameResult,4> vehicle_views_;
  std::array<VisionFrameResult,4> roadblock_views_;
  std::array<VisionFrameResult,4> crosswalk_views_;
  adas::GeometryObservation last_geometry_;
  adas::SignalObservation last_signal_;
  adas::TurnSignal requested_turn_ = adas::TurnSignal::None;
  uint64_t signal_count_ = 0;
  bool front_only_ = false;
  QPixmap rendered_;
  QCheckBox *mosaic_check_ = nullptr, *front_check_ = nullptr;
  QCheckBox *manual_stop_check_ = nullptr, *force_drive_check_ = nullptr;
  QCheckBox *auto_check_ = nullptr, *intersection_check_ = nullptr;
  QComboBox *lane_combo_ = nullptr, *signal_combo_ = nullptr, *force_lane_combo_ = nullptr, *nav_combo_ = nullptr;
  QDoubleSpinBox *speed_spin_ = nullptr, *limit_spin_ = nullptr;
  QLabel *decision_label_ = nullptr, *alarm_label_ = nullptr, *speed_label_ = nullptr, *source_label_ = nullptr;
  QLabel *stats_label_ = nullptr, *capability_label_ = nullptr, *calibration_label_ = nullptr;
};
#endif
