#ifndef CAMERA_H
#define CAMERA_H

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include "params.h"

namespace opencv_camera {

class Camera {
 public:
  Camera(CameraParams params);
  ~Camera();
  Camera(const Camera&) = delete;
  Camera& operator=(const Camera&) = delete;

  sensor_msgs::CameraInfo GetDefaultInfo() const;
  sensor_msgs::ImagePtr Capture();

  const CameraParams& GetParams() const;

 private:
  void MakeDefaultInfo();
  bool IsFrameBlurred() const;
  void DenoiseFrame();

 private:
  CameraParams params_;
  cv::VideoCapture capture_;
  cv::Mat frame_;
  cv::Mat frame_laplacian_;
  cv::Mat mean_;
  cv::Mat stddev_;
  sensor_msgs::ImagePtr msg_;
  sensor_msgs::CameraInfo cam_info_msg_;
};

}  // namespace opencv_camera

#endif  // CAMERA_H
