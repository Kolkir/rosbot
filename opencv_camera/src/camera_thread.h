#ifndef CAMERATHREAD_H
#define CAMERATHREAD_H

#include <boost/lockfree/spsc_queue.hpp>
#include <thread>
#include "camera.h"

namespace opencv_camera {

class CameraThread {
 public:
  CameraThread(std::unique_ptr<Camera> camera);
  ~CameraThread();
  CameraThread(const CameraThread&) = delete;
  CameraThread& operator=(const CameraThread&) = delete;

  void Start();
  void Stop();

  sensor_msgs::ImagePtr GetFrame();

 private:
  void ThreadFunc();

 private:
  std::unique_ptr<Camera> camera_;
  using QueueType = boost::lockfree::spsc_queue<sensor_msgs::ImagePtr>;
  std::unique_ptr<QueueType> queue_;
  std::thread thread_;
  std::atomic_bool stop_flag_;
};

}  // namespace opencv_camera

#endif  // CAMERATHREAD_H
