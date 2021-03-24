#ifndef CAMERANODELET_H
#define CAMERANODELET_H

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include "camera.h"
#include "camera_thread.h"

namespace opencv_camera {

class CameraNodelet : public nodelet::Nodelet {
 public:
  CameraNodelet();
  ~CameraNodelet() override;

 private:
  // Nodelet interface
  void onInit() override;

  void TimerCallback(const ros::TimerEvent& event);

 private:
  std::shared_ptr<ros::NodeHandle> node_handle_;
  std::shared_ptr<ros::NodeHandle> private_node_handle_;

  image_transport::CameraPublisher frame_publisher_;
  sensor_msgs::CameraInfo cam_info_msg_;
  ros::Timer timer_;
  std::unique_ptr<CameraThread> camera_thread_;
};

}  // namespace opencv_camera

#endif  // CAMERANODELET_H
