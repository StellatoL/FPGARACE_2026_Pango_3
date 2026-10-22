#ifndef ADAS_FILE_VIDEO_SOURCE_H
#define ADAS_FILE_VIDEO_SOURCE_H

#include "common/frame_packet.h"
#include <QString>
#include <QThread>
#include <functional>
#include <memory>

namespace adas {
// The decoder and its capture handle live exclusively in run(). The producer
// owns a separate pool; callbacks receive immutable retained pixels.
class FileVideoSource : public QThread {
  Q_OBJECT
public:
  explicit FileVideoSource(QObject *parent = nullptr);
  ~FileVideoSource() override;
  void set_callbacks(std::function<void(FramePacket)> on_frame,
                     std::function<uint64_t()> new_generation);
  void start_file(QString path, uint64_t generation, Layout layout);
  void stop();
  void set_paused(bool paused);
  void set_loop(bool loop);
  void set_layout(Layout layout); // Applied only while stopped.
  void seek(double seconds);
  double position_seconds() const;
  double duration_seconds() const;
  double native_fps() const;
signals:
  void opened(double fps, double duration, bool image, uint64_t generation);
  void error(QString message, uint64_t generation);
  void ended(uint64_t generation);
protected:
  void run() override;
private:
  struct State;
  std::unique_ptr<State> state_;
};
} // namespace adas
#endif
