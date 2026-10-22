#ifndef PLATE_RECOG_TASK_H
#define PLATE_RECOG_TASK_H

#include <QMutex>
#include <QRunnable>

#include <memory>

#include <opencv2/core/mat.hpp>

#include "inference/plate_infer_thread.h"
#include "core/plate_recognizer.h"
#include "inference/rga_preprocessor.h"

// QRunnable task: single-plate LPRNet recognition dispatched to a thread pool.
//
// Each task:
// 1. Crop+resize the plate ROI → 94×24 BGR (RGA or CPU)
// 2. Call PlateRecognizer::recognize_single_plate() for NPU inference
// 3. Notify PlateInferThread of completion (thread-safe, via QMutex)
// 4. Auto-delete after execution (setAutoDelete(true) by default)
class PlateRecogTask : public QRunnable {
public:
  PlateRecogTask() = delete;

  PlateRecogTask(std::shared_ptr<const cv::Mat> bgr_frame,
                 cv::Rect roi, float detection_confidence,
                 int left, int top, int right, int bottom,
                 vision::RgaPreprocessor *rga,
                 PlateRecognizer *recognizer,
                 PlateInferThread *owner,
                 PlateInferThread::RecogBatch *batch);

  void run() override;

private:
  std::shared_ptr<const cv::Mat> bgr_frame_;
  cv::Rect roi_;
  float detection_confidence_;
  int left_, top_, right_, bottom_;

  vision::RgaPreprocessor *rga_;       // non-owning
  PlateRecognizer *recognizer_;        // non-owning, per-worker instance
  PlateInferThread *owner_;            // non-owning, for completion callback
  PlateInferThread::RecogBatch *batch_; // non-owning, shared batch state

  // Per-task working buffer (94×24 BGR)
  cv::Mat plate_input_;
};

#endif // PLATE_RECOG_TASK_H
