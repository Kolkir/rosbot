#ifndef PARAMS_H
#define PARAMS_H

#include <ros/ros.h>

namespace opencv_camera {
struct CameraParams {
  int camera_index = 0;
  std::string name;
  std::string info_file_url;
  double frame_width = 0.0;
  double frame_height = 0.0;
  double frame_rate = 0.0;
  int buffer_size = 100;
  int out_queue_size = 30;
  double denoise_factor = 10.0;
  double blur_threshold = 800.0;
};

CameraParams ReadCameraParams(ros::NodeHandle& node_handle);
}  // namespace opencv_camera

#endif  // PARAMS_H
