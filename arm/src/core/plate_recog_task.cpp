// QRunnable for single-plate LPRNet recognition dispatched to a thread pool.
//
// Each task:
// 1. Crop+resize plate ROI → 94×24 BGR (RGA hardware or CPU fallback)
// 2. Call PlateRecognizer::recognize_single_plate() for NPU inference + CTC decode
// 3. Notify PlateInferThread of completion (thread-safe via RecogBatch)
// 4. Auto-delete after execution (default QRunnable behaviour)

#include "core/plate_recog_task.h"

#include <QMutexLocker>

#include <opencv2/imgproc.hpp>

#include "inference/plate_infer_thread.h"

PlateRecogTask::PlateRecogTask(
    std::shared_ptr<const cv::Mat> bgr_frame,
    cv::Rect roi, float detection_confidence,
    int left, int top, int right, int bottom,
    vision::RgaPreprocessor *rga,
    PlateRecognizer *recognizer,
    PlateInferThread *owner,
    PlateInferThread::RecogBatch *batch)
    : bgr_frame_(std::move(bgr_frame)),
      roi_(roi),
      detection_confidence_(detection_confidence),
      left_(left), top_(top), right_(right), bottom_(bottom),
      rga_(rga),
      recognizer_(recognizer),
      owner_(owner),
      batch_(batch) {}

void PlateRecogTask::run() {
  // Validate inputs
  if (bgr_frame_ == nullptr || bgr_frame_->empty()) {
    owner_->on_plate_recog_done(batch_, PlateDetectionResult{});
    return;
  }

  // --- Step 1: crop + resize ROI → 94×24 BGR ---
  plate_input_.create(24, 94, CV_8UC3);

  {
    const int rx = std::max(0, roi_.x);
    const int ry = std::max(0, roi_.y);
    const int rw = std::min(roi_.width, bgr_frame_->cols - rx);
    const int rh = std::min(roi_.height, bgr_frame_->rows - ry);

    if (rw <= 2 || rh <= 2) {
      owner_->on_plate_recog_done(batch_, PlateDetectionResult{});
      return;
    }

    if (rga_ != nullptr && rga_->is_available()) {
      rga_->crop_resize(*bgr_frame_, plate_input_,
                        cv::Rect(rx, ry, rw, rh), 94, 24);
    } else {
      cv::Mat roi_view = (*bgr_frame_)(cv::Rect(rx, ry, rw, rh));
      cv::resize(roi_view, plate_input_, cv::Size(94, 24), 0, 0,
                 cv::INTER_LINEAR);
    }
  }

  // --- Step 2: LPRNet inference + CTC decode ---
  PlateDetectionResult result;
  result.left = left_;
  result.top = top_;
  result.right = right_;
  result.bottom = bottom_;
  result.confidence = detection_confidence_;

  if (recognizer_ != nullptr) {
    result.text = recognizer_->recognize_single_plate(plate_input_);
  }

  // --- Step 3: notify owner ---
  owner_->on_plate_recog_done(batch_, std::move(result));
}
