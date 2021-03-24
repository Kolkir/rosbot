#include "camera_nodelet.h"

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <nodelet/loader.h>
#include <ros/ros.h>
#include "params.h"

namespace opencv_camera {
CameraNodelet::CameraNodelet() {}

CameraNodelet::~CameraNodelet() {
  if (camera_thread_)
    camera_thread_->Stop();
  timer_.stop();
}

void CameraNodelet::onInit() {
  node_handle_.reset(new ros::NodeHandle(getNodeHandle()));
  private_node_handle_.reset(new ros::NodeHandle(getPrivateNodeHandle()));

  CameraParams params = ReadCameraParams(*node_handle_);
  auto camera = std::make_unique<Camera>(params);
  auto camera_ptr = camera.get();
  camera_thread_ = std::make_unique<CameraThread>(std::move(camera));

  // CameraInfoManager is responsible for SetCameraInfo service requests, saves
  // and restores sensor_msgs/CameraInfo data.
  camera_info_manager::CameraInfoManager cam_info_manager(
      *node_handle_, params.name, params.info_file_url);
  cam_info_msg_ = cam_info_manager.getCameraInfo();

  // Create a default camera info if we didn't get a stored one on
  // initialization
  if (cam_info_msg_.distortion_model == "") {
    NODELET_WARN_STREAM(
        "No calibration file given, publishing a reasonable default camera "
        "info.");
    cam_info_msg_ = camera_ptr->GetDefaultInfo();
  }

  // Image_transport is responsible for publishing and subscribing to Images
  image_transport::ImageTransport image_transport(*node_handle_);

  // Publish to the /camera topic
  uint32_t queue_size = static_cast<uint32_t>(params.out_queue_size);
  frame_publisher_ =
      image_transport.advertiseCamera("/opencv_camera/image_raw", queue_size);

  ros::Rate frame_rate(ros::Duration(1.0 / params.frame_rate));

  camera_thread_->Start();

  timer_ = node_handle_->createTimer(frame_rate, &CameraNodelet::TimerCallback,
                                     this);
}

void CameraNodelet::TimerCallback(const ros::TimerEvent& /*event*/) {
  auto msg = camera_thread_->GetFrame();
  if (msg) {
    frame_publisher_.publish(*msg, cam_info_msg_, msg->header.stamp);
  }
}
}  // namespace opencv_camera
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_camera::CameraNodelet, nodelet::Nodelet)
