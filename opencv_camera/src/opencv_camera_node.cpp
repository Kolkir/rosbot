#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include "camera.h"
#include "camera_thread.h"
#include "params.h"

int main(int argc, char** argv) {
  using namespace opencv_camera;
  ros::init(argc, argv, "opencv_camera");
  ros::NodeHandle node_handler;

  CameraParams params = ReadCameraParams(node_handler);
  auto camera = std::make_unique<Camera>(params);
  auto camera_ptr = camera.get();
  auto camera_thread = std::make_unique<CameraThread>(std::move(camera));

  // CameraInfoManager is responsible for SetCameraInfo service requests, saves
  // and restores sensor_msgs/CameraInfo data.
  camera_info_manager::CameraInfoManager cam_info_manager(
      node_handler, params.name, params.info_file_url);
  sensor_msgs::CameraInfo cam_info_msg = cam_info_manager.getCameraInfo();

  // Image_transport is responsible for publishing and subscribing to Images
  image_transport::ImageTransport it(node_handler);

  // Publish to the /camera topic
  image_transport::CameraPublisher pub_frame =
      it.advertiseCamera("/opencv_camera/image_raw", 1);

  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(ros::Duration(1.0 / params.frame_rate));

  camera_thread->Start();
  while (node_handler.ok()) {
    msg = camera_thread->GetFrame();
    if (!msg)
      continue;
    // Create a default camera info if we didn't get a stored one on
    // initialization
    if (cam_info_msg.distortion_model == "") {
      ROS_WARN_STREAM(
          "No calibration file given, publishing a reasonable default camera "
          "info.");
      cam_info_msg = camera_ptr->GetDefaultInfo();
    }
    pub_frame.publish(*msg, cam_info_msg, ros::Time::now());

    ros::spinOnce();
    loop_rate.sleep();
  }
}
