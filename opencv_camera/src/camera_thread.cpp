#include "camera_thread.h"

namespace opencv_camera {

static auto consumer_timeout = std::chrono::milliseconds(10);

CameraThread::CameraThread(std::unique_ptr<Camera> camera)
    : camera_(std::move(camera)), stop_flag_(false) {
  queue_ = std::make_unique<QueueType>(camera_->GetParams().buffer_size);
}

CameraThread::~CameraThread() {
  Stop();
}

void CameraThread::Start() {
  thread_ = std::thread([this]() { ThreadFunc(); });
}

void CameraThread::Stop() {
  stop_flag_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
}

sensor_msgs::ImagePtr CameraThread::GetFrame() {
  sensor_msgs::ImagePtr img_msg;
  queue_->pop(img_msg);
  return img_msg;
}

void CameraThread::ThreadFunc() {
  bool done = false;
  while (!done) {
    auto img_msg = camera_->Capture();
    if (img_msg) {
      while (!queue_->push(img_msg)) {
        std::this_thread::sleep_for(consumer_timeout);
      }
    }
    done = stop_flag_.load();
  }
}

}  // namespace opencv_camera
