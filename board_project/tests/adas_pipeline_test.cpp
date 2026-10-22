#include "inference/adas_pipeline.h"
#include "core/view_tracker.h"
#include "core/traffic_scene_detector.h"
#include "core/plate_recognizer.h"
#include <QCoreApplication>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

// The workers use real scheduling, ownership and postprocessing. Only the NPU
// entry points are substituted so this executable runs without loading a model.
namespace {
std::mutex backend_mutex;
std::condition_variable backend_cv;
bool block_backend = false, entered_backend = false, backend_ready = true;
bool used_legacy = true;
bool used_vehicle_in_primary = true, block_vehicle = false, entered_vehicle = false;
int backend_calls = 0, backend_initializations = 0;
int vehicle_calls = 0, legacy_resets = 0;
int vehicle_width = 0, vehicle_height = 0, vehicle_stride = 0;
adas::Status vehicle_backend_status = adas::Status::Valid;
int observed_stride = 0, observed_width = 0, observed_height = 0;
unsigned char observed_first = 0, observed_next_row = 0;
std::vector<TrafficSceneDetection> detections;
std::vector<TrafficSceneDetection> vehicle_detections{{10,10,10,30,35,.9f}, {10,50,10,90,40,.8f},
  {10,5,55,25,85,.8f}, {10,80,60,110,90,.8f}};

template<class T> struct Results {
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<T> values;
  void add(T value) {
    std::lock_guard<std::mutex> lock(mutex);
    values.push_back(std::move(value)); cv.notify_all();
  }
  bool wait(size_t count) {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, std::chrono::seconds(4), [&] { return values.size() >= count; });
  }
  size_t size() { std::lock_guard<std::mutex> lock(mutex); return values.size(); }
};
adas::FramePacket frame(uint64_t generation, uint64_t sequence,
                        adas::Layout layout = adas::Layout::Mosaic, bool image = false) {
  const int w = 128, h = 96, stride = 272;
  auto storage = std::make_shared<adas::FrameStorage>(stride * h);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      int view = (y >= h / 2 ? 2 : 0) + (x >= w / 2 ? 1 : 0);
      storage->bytes[y * stride + x * 2] = static_cast<unsigned char>(20 + view + y);
      storage->bytes[y * stride + x * 2 + 1] = 0;
    }
  return {storage, {generation, sequence, double(sequence) / 30, adas::monotonic_seconds(), image},
          w, h, stride, layout};
}
adas::ObjectObservation observation(uint64_t generation, uint64_t sequence,
                                  adas::ViewId view = adas::ViewId::Front) {
  adas::ObjectObservation o;
  o.stamp = {generation, sequence, double(sequence) / 30, adas::monotonic_seconds(), false};
  o.view = view; o.traffic_status = adas::Status::Valid;
  o.obstacles.push_back({1, -1, {10, 10, 30, 40}, .9f, -1});
  return o;
}
void tracker_test() {
  adas::ViewTracker tracker(2);
  auto a = observation(1, 1); assert(tracker.update(a));
  const auto first = a.obstacles[0].track_id; assert(first >= 0);
  auto b = observation(1, 2); b.obstacles[0].box.left += 2;
  assert(tracker.update(b) && b.obstacles[0].track_id == first);
  auto other_view = observation(1, 2, adas::ViewId::Right);
  assert(tracker.update(other_view) && other_view.obstacles[0].track_id != first);
  auto other_class = observation(1, 3); other_class.obstacles[0].class_id = 10;
  assert(tracker.update(other_class) && other_class.obstacles[0].track_id != first);
  auto repeated = observation(1, 3); assert(!tracker.update(repeated));
  for (uint64_t seq = 4; seq <= 6; ++seq) {
    auto empty = observation(1, seq); empty.obstacles.clear(); assert(tracker.update(empty));
  }
  auto expired = observation(1, 7); assert(tracker.update(expired));
  assert(expired.obstacles[0].track_id != first);
  auto fresh = observation(2, 1); assert(tracker.update(fresh));
  assert(fresh.obstacles[0].track_id >= 0);
  auto stale = observation(1, 8); assert(!tracker.update(stale));
}
void partial_tracker_test() {
  adas::ViewTracker tracker(2);
  auto person = observation(1, 1); person.components = adas::ObjectObservation::Traffic;
  assert(tracker.update(person)); const auto person_id = person.obstacles[0].track_id;
  auto vehicle = observation(1, 1); vehicle.components = adas::ObjectObservation::Vehicle;
  vehicle.obstacles[0].class_id = 10; vehicle.vehicle_status = adas::Status::Valid;
  assert(tracker.update(vehicle)); const auto vehicle_id = vehicle.obstacles[0].track_id;
  assert(person_id != vehicle_id);
  assert(!tracker.update(vehicle));
  for (uint64_t sequence = 2; sequence < 10; ++sequence) {
    auto update = observation(1, sequence); update.components = adas::ObjectObservation::Traffic;
    assert(tracker.update(update) && update.obstacles[0].track_id == person_id);
  }
  vehicle.stamp.sequence = 10;
  assert(tracker.update(vehicle) && vehicle.obstacles[0].track_id == vehicle_id);
  auto older_person = observation(1, 8); older_person.components = adas::ObjectObservation::Traffic;
  assert(!tracker.update(older_person));
  for (uint64_t sequence = 11; sequence < 14; ++sequence) {
    auto empty = vehicle; empty.stamp.sequence = sequence; empty.obstacles.clear(); assert(tracker.update(empty));
  }
  vehicle.stamp.sequence = 14;
  assert(tracker.update(vehicle) && vehicle.obstacles[0].track_id != vehicle_id);
}
void front_mask_cache_test() {
  adas::FrontObjectCache cache;
  auto traffic = observation(1, 10); traffic.components = adas::ObjectObservation::Traffic;
  traffic.width = 64; traffic.height = 48; traffic.stamp.source_s = 1;
  auto vehicle = traffic; vehicle.components = adas::ObjectObservation::Vehicle;
  vehicle.vehicle_status = adas::Status::Valid; vehicle.obstacles[0].class_id = 10;
  vehicle.stamp.sequence = 8; vehicle.stamp.source_s = .9;
  assert(cache.update(traffic) && cache.update(vehicle));
  assert(cache.boxes(traffic.stamp, 64, 48).size() == 2);
  auto later = traffic.stamp; later.source_s = 1.2501;
  assert(cache.boxes(later, 64, 48).size() == 1); // Person capture age exceeds 250 ms.
  later.source_s = 1.649;
  assert(cache.boxes(later, 64, 48).size() == 1); // Vehicle remains within its 750 ms budget.
  later.source_s = 1.651; assert(cache.boxes(later, 64, 48).empty());
  traffic.stamp.sequence = 11; traffic.stamp.source_s = 1.2; assert(cache.update(traffic));
  assert(cache.boxes(traffic.stamp, 64, 48).size() == 2);
  traffic.stamp.sequence = 12; traffic.stamp.source_s = 1.8; assert(cache.update(traffic));
  assert(cache.boxes(traffic.stamp, 64, 48).size() == 1); // Traffic did not refresh the vehicle clock.
  assert(cache.boxes(traffic.stamp, 128, 96).empty());
  traffic.stamp.sequence = 13; traffic.traffic_status = adas::Status::Failed; traffic.obstacles.clear();
  assert(cache.update(traffic) && cache.boxes(traffic.stamp, 64, 48).empty());
  vehicle.stamp = {2, 1, .1, 10, false}; assert(cache.update(vehicle));
  assert(cache.boxes(vehicle.stamp, 64, 48).size() == 1);
  assert(!cache.update(traffic));
  cache.clear(); assert(cache.boxes(vehicle.stamp, 64, 48).empty());
}
void detection_test() {
  Results<VisionFrameResult> results;
  Results<VisionFrameResult> vehicles;
  adas::DetectionWorker worker;
  worker.set_model_root("/tmp/test-model-root");
  QObject::connect(&worker, &adas::DetectionWorker::result_ready, &worker,
                   [&](VisionFrameResult v) {
    if (v.mode == DetectionMode::Adas && v.observation.components == adas::ObjectObservation::Vehicle) vehicles.add(std::move(v));
    else results.add(std::move(v));
  }, Qt::DirectConnection);
  {
    std::lock_guard<std::mutex> lock(backend_mutex);
    detections = {{1, 10, 10, 30, 40, .9f}, {3, 2, 2, 5, 8, .8f},
                  {5, 6, 2, 9, 8, .8f}, {2, 10, 2, 13, 8, .8f}};
  }
  std::array<adas::Roi, 3> rois{{{0, 0, .5, .5}, {.5, 0, .5, .5}, {0, .5, 1, .5}}};
  adas::GroundCalibration calibration;
  assert(calibration.configure({{{0,0}, {63,0}, {63,47}, {0,47}}}, 10, 20, 5, 64, 48));
  worker.configure(calibration, rois);
  worker.start();
  const int expected[] = {0, 0, 1, 0, 2, 0, 3};
  int64_t front_track = -1;
  for (int i = 0; i < 7; ++i) {
    auto input = frame(1, i + 1);
    const auto stamp = input.stamp;
    worker.submit(input);
    assert(results.wait(i + 1));
    auto r = results.values[i];
    assert(static_cast<int>(r.view) == expected[i]);
    assert(r.mode == DetectionMode::Adas && r.observation.width == 64 && r.observation.height == 48);
    assert(r.observation.components == adas::ObjectObservation::Traffic && r.observation.vehicle_status == adas::Status::Missing);
    assert(r.observation.obstacles.size() == 1 && r.observation.obstacles[0].class_id == 1);
    assert(r.stamp.source_s == stamp.source_s && r.stamp.receive_s == stamp.receive_s);
    assert(r.observation.stamp.receive_s == stamp.receive_s);
    assert(r.observation.has_lane_lights && r.observation.lane_lights[0] == adas::LightState::Red);
    assert(r.observation.lane_lights[1] == adas::LightState::Unknown);
    if (expected[i] == 0) {
      if (front_track < 0) front_track = r.observation.obstacles[0].track_id;
      assert(r.observation.obstacles[0].track_id == front_track);
      assert(r.observation.obstacles[0].distance_m > 0);
    } else {
      assert(r.observation.obstacles[0].distance_m < 0);
    }
    std::lock_guard<std::mutex> lock(backend_mutex);
    assert(!used_legacy && !used_vehicle_in_primary && observed_stride == 272 && observed_width == 64 && observed_height == 48);
    assert(observed_first == 20 + expected[i] + (expected[i] >= 2 ? 48 : 0));
    assert(observed_next_row == observed_first + 1);
  }
  assert(vehicles.wait(4));
  { std::lock_guard<std::mutex> lock(backend_mutex); assert(vehicle_width == 128 && vehicle_height == 96 && vehicle_stride == 272); }
  worker.submit(frame(1, 7));
  worker.submit(frame(1, 6));
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  assert(results.size() == 7);
  worker.submit(frame(2, 1, adas::Layout::Mosaic, true));
  assert(results.wait(11));
  assert(vehicles.wait(8));
  for (int i = 0; i < 4; ++i) assert(static_cast<int>(results.values[7 + i].view) == i);
  for (int i = 0; i < 4; ++i) {
    const auto &v = vehicles.values[4+i];
    assert(v.stamp.generation == 2 && v.stamp.sequence == 1 && v.stamp.static_image);
    assert(static_cast<int>(v.view) == i && v.observation.components == adas::ObjectObservation::Vehicle);
    assert(v.observation.traffic_status == adas::Status::Missing && v.observation.vehicle_status == adas::Status::Valid);
    assert(v.observation.width == 64 && v.observation.height == 48 && v.observation.obstacles.size() == 1);
    assert(v.overlays.size() == 1 && v.overlays[0].class_id == 10);
    const auto &box = v.observation.obstacles[0].box;
    assert(box.left >= 0 && box.top >= 0 && box.right <= 64 && box.bottom <= 48);
    if (i == 1) assert(box.left == 0 && box.right == 26 && box.top == 10);
    if (i == 2) assert(box.left == 5 && box.top == 7);
  }
  worker.submit(frame(2, 1, adas::Layout::Mosaic, true));
  std::this_thread::sleep_for(std::chrono::milliseconds(40)); assert(results.size() == 11);
  {
    std::lock_guard<std::mutex> lock(backend_mutex); backend_ready = false;
  }
  worker.submit(frame(3, 1, adas::Layout::Single)); assert(results.wait(12));
  assert(results.values[11].observation.traffic_status == adas::Status::Failed);
  {
    std::lock_guard<std::mutex> lock(backend_mutex); backend_ready = true;
  }
  worker.submit(frame(4, 1, adas::Layout::Single)); assert(results.wait(13));
  assert(results.values[12].observation.width == 128 && results.values[12].observation.obstacles[0].distance_m < 0);
  worker.set_mode(DetectionMode::RawVideo); worker.submit(frame(5, 1));
  std::this_thread::sleep_for(std::chrono::milliseconds(40)); assert(results.size() == 13);
  worker.set_mode(DetectionMode::TrafficScene); worker.submit(frame(6, 1)); assert(results.wait(14));
  assert(results.values[13].mode == DetectionMode::TrafficScene && results.values[13].observation.width == 128);
  { std::lock_guard<std::mutex> lock(backend_mutex); assert(used_legacy && used_vehicle_in_primary); }
  worker.set_mode(DetectionMode::PlateRecognition); worker.submit(frame(7, 1)); assert(results.wait(15));
  assert(results.values[14].mode == DetectionMode::PlateRecognition);
  assert(results.values[14].overlays.size() == 1 && results.values[14].overlays[0].color_g == 200);
  worker.stop(); assert(!worker.isRunning());
  assert(backend_initializations == 1);
  assert(legacy_resets == 6);
}
void pending_and_ownership_test() {
  Results<VisionFrameResult> results;
  adas::DetectionWorker worker;
  QObject::connect(&worker, &adas::DetectionWorker::result_ready, &worker,
                   [&](VisionFrameResult v) {
    if (v.observation.components == adas::ObjectObservation::Traffic) results.add(std::move(v));
  }, Qt::DirectConnection);
  {
    std::lock_guard<std::mutex> lock(backend_mutex);
    backend_ready = true; block_backend = true; entered_backend = false;
  }
  worker.start();
  auto owned = frame(4, 1); std::weak_ptr<const adas::FrameStorage> retained = owned.pixels;
  worker.submit(owned); owned = {};
  {
    std::unique_lock<std::mutex> lock(backend_mutex);
    assert(backend_cv.wait_for(lock, std::chrono::seconds(2), [] { return entered_backend; }));
  }
  assert(!retained.expired());
  worker.submit(frame(4, 2)); worker.submit(frame(4, 3));
  assert(worker.dropped_frames() == 1);
  {
    std::lock_guard<std::mutex> lock(backend_mutex); block_backend = false; backend_cv.notify_all();
  }
  assert(results.wait(2));
  assert(results.values[1].stamp.sequence == 3 && results.values[1].view == adas::ViewId::Front);
  worker.stop(); assert(retained.expired());
}
void multi_rate_test() {
  Results<VisionFrameResult> primary, vehicles;
  adas::DetectionWorker worker;
  QObject::connect(&worker, &adas::DetectionWorker::result_ready, &worker,
    [&](VisionFrameResult result) {
      if (result.observation.components == adas::ObjectObservation::Traffic) primary.add(std::move(result));
      else vehicles.add(std::move(result));
    }, Qt::DirectConnection);
  int initializations, initial_vehicle_calls;
  {
    std::lock_guard<std::mutex> lock(backend_mutex);
    block_vehicle = true; entered_vehicle = false;
    initializations = backend_initializations; initial_vehicle_calls = vehicle_calls;
  }
  worker.start(); auto input = frame(20, 1); const auto stamp = input.stamp;
  std::weak_ptr<const adas::FrameStorage> retained = input.pixels; worker.submit(input); input = {};
  assert(primary.wait(1));
  {
    std::unique_lock<std::mutex> lock(backend_mutex);
    assert(backend_cv.wait_for(lock, std::chrono::seconds(2), [] { return entered_vehicle; }));
  }
  assert(vehicles.size() == 0 && primary.size() == 1 && !retained.expired());
  worker.submit(frame(20, 2)); worker.submit(frame(20, 3));
  {
    std::lock_guard<std::mutex> lock(backend_mutex); block_vehicle = false; backend_cv.notify_all();
  }
  assert(vehicles.wait(4) && primary.wait(2));
  for (const auto &result : vehicles.values) {
    assert(result.stamp.sequence == 1 && result.stamp.source_s == stamp.source_s && result.stamp.receive_s == stamp.receive_s);
    assert(result.observation.stamp.sequence == 1 && result.observation.components == adas::ObjectObservation::Vehicle);
  }
  const int expected[] = {0,0,1,0,2,0,3};
  for (int job = 2; job < 15; ++job) {
    worker.submit(frame(20, job + 2)); assert(primary.wait(job + 1));
    assert(static_cast<int>(primary.values[job].view) == expected[job % 7]);
  }
  assert(vehicles.wait(8)); worker.stop();
  assert(backend_initializations == initializations + 1 && vehicle_calls == initial_vehicle_calls + 2);
}
void vehicle_availability_test() {
  Results<VisionFrameResult> primary, vehicles;
  adas::DetectionWorker worker;
  QObject::connect(&worker, &adas::DetectionWorker::result_ready, &worker,
    [&](VisionFrameResult result) {
      if (result.observation.components == adas::ObjectObservation::Traffic) primary.add(std::move(result));
      else vehicles.add(std::move(result));
    }, Qt::DirectConnection);
  worker.start();
  const adas::Status states[] = {adas::Status::Missing, adas::Status::Failed, adas::Status::Valid};
  for (int i = 0; i < 3; ++i) {
    {
      std::lock_guard<std::mutex> lock(backend_mutex); vehicle_backend_status = states[i];
      if (i == 2) vehicle_detections.clear();
    }
    worker.submit(frame(40 + i, 1)); assert(primary.wait(i + 1) && vehicles.wait((i + 1) * 4));
    assert(primary.values[i].observation.traffic_status == adas::Status::Valid);
    for (int view = 0; view < 4; ++view) {
      const auto &result = vehicles.values[i * 4 + view];
      assert(result.observation.vehicle_status == states[i] && result.observation.obstacles.empty());
      assert(result.model_ready == (states[i] == adas::Status::Valid));
    }
  }
  worker.stop();
}
void geometry_test() {
  Results<adas::GeometryObservation> results;
  adas::GeometryWorker worker;
  QObject::connect(&worker, &adas::GeometryWorker::result_ready, &worker,
                   [&](adas::GeometryObservation v) { results.add(std::move(v)); }, Qt::DirectConnection);
  worker.configure({}, {}); worker.set_enabled(true); worker.start();
  const auto packet = frame(5, 1, adas::Layout::Mosaic, true);
  worker.submit(packet); assert(results.wait(1));
  assert(results.values[0].width == 64 && results.values[0].height == 48);
  assert(results.values[0].stamp.source_s == packet.stamp.source_s);
  assert(!results.values[0].speed.valid);
  worker.submit(packet); std::this_thread::sleep_for(std::chrono::milliseconds(80)); assert(results.size() == 1);
  worker.set_enabled(false); worker.submit(frame(6, 1));
  std::this_thread::sleep_for(std::chrono::milliseconds(80)); assert(results.size() == 1);
  worker.set_enabled(true); worker.submit(frame(6, 2)); assert(results.wait(2));
  worker.submit(frame(6, 3)); worker.submit(frame(6, 4)); worker.submit(frame(6, 5));
  assert(results.wait(3)); assert(results.values.back().stamp.sequence == 5);
  assert(worker.dropped_frames() >= 2);
  worker.stop(); assert(!worker.isRunning());
}
} // namespace

TrafficSceneDetector::TrafficSceneDetector() = default;
TrafficSceneDetector::~TrafficSceneDetector() = default;
bool TrafficSceneDetector::initialize(const std::string &) { ++backend_initializations; return true; }
void TrafficSceneDetector::reset_tracking() { ++legacy_resets; }
VisionFrameResult TrafficSceneDetector::infer_rgb565(const unsigned char *data, int w, int h, int stride, bool legacy, bool run_vehicle) {
  std::unique_lock<std::mutex> lock(backend_mutex);
  ++backend_calls; used_legacy = legacy; used_vehicle_in_primary = run_vehicle;
  entered_backend = true; backend_cv.notify_all();
  backend_cv.wait(lock, [] { return !block_backend; });
  observed_stride = stride; observed_width = w; observed_height = h;
  observed_first = data[0]; observed_next_row = data[stride];
  VisionFrameResult r; r.model_ready = backend_ready; r.detections = detections;
  r.observation.components = run_vehicle ? adas::ObjectObservation::Combined : adas::ObjectObservation::Traffic;
  r.observation.traffic_status = adas::Status::Valid;
  r.observation.vehicle_status = run_vehicle ? adas::Status::Valid : adas::Status::Missing;
  for (auto d : detections) if (d.class_id == 1 || (d.class_id == 10 && run_vehicle))
    r.observation.obstacles.push_back({d.class_id, -1,
      {float(d.left), float(d.top), float(d.right), float(d.bottom)}, d.confidence, -1});
  return r;
}
VisionFrameResult TrafficSceneDetector::infer_vehicle_rgb565(const unsigned char *, int w, int h, int stride) {
  std::unique_lock<std::mutex> lock(backend_mutex);
  ++vehicle_calls; vehicle_width = w; vehicle_height = h; vehicle_stride = stride;
  entered_vehicle = true; backend_cv.notify_all(); backend_cv.wait(lock, [] { return !block_vehicle; });
  VisionFrameResult result; result.model_ready = backend_ready && vehicle_backend_status == adas::Status::Valid;
  result.detections = vehicle_detections;
  result.observation.components = adas::ObjectObservation::Vehicle;
  result.observation.vehicle_status = backend_ready ? vehicle_backend_status : adas::Status::Failed;
  for (auto d : vehicle_detections) result.observation.obstacles.push_back({10, -1,
    {float(d.left), float(d.top), float(d.right), float(d.bottom)}, d.confidence, -1});
  return result;
}
PlateRecognizer::PlateRecognizer() = default;
PlateRecognizer::~PlateRecognizer() = default;
bool PlateRecognizer::initialize(const std::string &) { return true; }
PlateFrameResult PlateRecognizer::infer_rgb565(const unsigned char *, int, int, int) {
  PlateFrameResult r; r.model_ready = true;
  r.plates.push_back({1, 2, 10, 12, .9f, "test", u8"蓝", u8"新能源"}); return r;
}

int main(int argc, char **argv) {
  QCoreApplication app(argc, argv);
  tracker_test(); partial_tracker_test(); front_mask_cache_test(); detection_test(); pending_and_ownership_test();
  multi_rate_test(); vehicle_availability_test(); geometry_test();
  std::cout << "ADAS tracking, scheduling, ownership, duplicate gates and geometry: PASS\n";
}
