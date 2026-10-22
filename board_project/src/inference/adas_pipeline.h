#ifndef ADAS_PIPELINE_H
#define ADAS_PIPELINE_H

#include "common/frame_packet.h"
#include "common/vision_result.h"
#include "core/ground_calibration.h"
#include <QMetaType>
#include <QString>
#include <QThread>
#include <memory>

#ifndef ADAS_VISION_RESULT_METATYPE
#define ADAS_VISION_RESULT_METATYPE
Q_DECLARE_METATYPE(VisionFrameResult)
#endif
Q_DECLARE_METATYPE(adas::GeometryObservation)

namespace adas {
class DetectionWorker : public QThread {
  Q_OBJECT
public:
  explicit DetectionWorker(QObject *parent = nullptr);
  ~DetectionWorker() override;
  void set_model_root(QString root);
  void set_mode(DetectionMode mode);
  void submit(FramePacket packet);
  void configure(GroundCalibration calibration, std::array<Roi, 3> light_rois, Roi navigation = {0,0,.32,.30});
  void set_intent(TurnSignal signal);
  void stop();
  uint64_t dropped_frames() const;
signals:
  void result_ready(VisionFrameResult result);
  void geometry_ready(adas::GeometryObservation result);
protected:
  void run() override;
private:
  struct State;
  std::unique_ptr<State> state_;
};

class GeometryWorker : public QThread {
  Q_OBJECT
public:
  explicit GeometryWorker(QObject *parent = nullptr);
  ~GeometryWorker() override;
  void set_enabled(bool enabled);
  void submit(FramePacket packet);
  void configure(GroundCalibration calibration, Roi nav_roi);
  void update_objects(ObjectObservation objects);
  void stop();
  uint64_t dropped_frames() const;
signals:
  void result_ready(adas::GeometryObservation result);
protected:
  void run() override;
private:
  struct State;
  std::unique_ptr<State> state_;
};
} // namespace adas
#endif
