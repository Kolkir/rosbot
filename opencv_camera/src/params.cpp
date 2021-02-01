#include "params.h"

namespace opencv_camera {
CameraParams ReadCameraParams(ros::NodeHandle& node_handler) {
  CameraParams params;

  if (!node_handler.param("/opencv_camera/buffer_size", params.buffer_size,
                          100)) {
    ROS_ERROR_STREAM("Using default buffer size");
  } else {
    ROS_WARN_STREAM("Camera buffer size = " << params.buffer_size);
  }

  if (!node_handler.param("/opencv_camera/out_queue_size", params.out_queue_size,
                          30)) {
    ROS_ERROR_STREAM("Using default queue size");
  } else {
    ROS_WARN_STREAM("Camera queue size = " << params.out_queue_size);
  }

  if (!node_handler.param("/opencv_camera/name", params.name,
                          std::string("opencv_camera"))) {
    ROS_ERROR_STREAM("Using default camera name");
  } else {
    ROS_WARN_STREAM("Camera name param = " << params.name);
  }

  if (!node_handler.param("/opencv_camera/info_file_url", params.info_file_url,
                          std::string())) {
    ROS_ERROR_STREAM("Using default camera info_file_url");
  } else {
    ROS_WARN_STREAM("Camera info_file_url param = " << params.info_file_url);
  }

  if (!node_handler.param("/opencv_camera/camera_index", params.camera_index,
                          0)) {
    ROS_ERROR_STREAM("Using default camera index param");
  } else {
    ROS_WARN_STREAM("Camera index param = " << params.camera_index);
  }

  if (!node_handler.param("/opencv_camera/frame_width", params.frame_width,
                          640.0)) {
    ROS_ERROR_STREAM("Using default frame width param");
  } else {
    ROS_WARN_STREAM("Frame width param = " << params.frame_width);
  }

  if (!node_handler.param("/opencv_camera/frame_height", params.frame_height,
                          480.0)) {
    ROS_ERROR_STREAM("Using default frame height param");
  } else {
    ROS_WARN_STREAM("Frame height param = " << params.frame_height);
  }

  if (!node_handler.param("/opencv_camera/frame_rate", params.frame_rate,
                          15.0)) {
    ROS_ERROR_STREAM("Using default frame rate param");
  } else {
    ROS_WARN_STREAM("Frame rate param = " << params.frame_rate);
  }

  return params;
}

}  // namespace opencv_camera
