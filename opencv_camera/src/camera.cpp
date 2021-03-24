#include "camera.h"
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <boost/assign/list_of.hpp>

namespace opencv_camera {

Camera::Camera(CameraParams params) : params_(params) {
  capture_ = cv::VideoCapture(params_.camera_index, cv::CAP_V4L2);
  if (!capture_.isOpened()) {
    ROS_ERROR_STREAM("Failed to open camera with index " << params_.camera_index
                                                         << "!");
    ros::shutdown();
    exit(1);
  }

  if (!capture_.set(cv::CAP_PROP_FRAME_WIDTH, params_.frame_width)) {
    ROS_ERROR_STREAM("Failed to set frame with " << params_.frame_width);
  }

  if (!capture_.set(cv::CAP_PROP_FRAME_HEIGHT, params_.frame_height)) {
    ROS_ERROR_STREAM("Failed to set frame height " << params_.frame_height);
  }

  if (!capture_.set(cv::CAP_PROP_FPS, params_.frame_rate)) {
    ROS_ERROR_STREAM("Failed to set frame rate " << params_.frame_rate);
  }

  Capture();

  ROS_WARN_STREAM("Frame width = " << frame_.cols
                                   << " height = " << frame_.rows);

  MakeDefaultInfo();
}

Camera::~Camera() {
  if (capture_.isOpened()) {
    capture_.release();
  }
}

sensor_msgs::CameraInfo Camera::GetDefaultInfo() const {
  return cam_info_msg_;
}

void Camera::MakeDefaultInfo() {
  cam_info_msg_.header.frame_id = msg_->header.frame_id;
  // Fill image size
  cam_info_msg_.height = msg_->height;
  cam_info_msg_.width = msg_->width;
  // Add the most common distortion model as sensor_msgs/CameraInfo says
  cam_info_msg_.distortion_model = "plumb_bob";
  // Don't let distorsion matrix be empty
  cam_info_msg_.D.resize(5, 0.0);
  // Give a reasonable default intrinsic camera matrix
  cam_info_msg_.K =
      boost::assign::list_of(msg_->width / 2.0)(0.0)(msg_->width / 2.0)(0.0)(
          msg_->height / 2.0)(msg_->height / 2.0)(0.0)(0.0)(1.0);
  // Give a reasonable default rectification matrix
  cam_info_msg_.R =
      boost::assign::list_of(1.0)(0.0)(0.0)(0.0)(1.0)(0.0)(0.0)(0.0)(1.0);
  // Give a reasonable default projection matrix
  cam_info_msg_.P =
      boost::assign::list_of(msg_->width / 2.0)(0.0)(msg_->width / 2.0)(0.0)(
          0.0)(msg_->height / 2.0)(msg_->height / 2.0)(0.0)(0.0)(0.0)(1.0)(0.0);
}

sensor_msgs::ImagePtr Camera::Capture() {
  capture_ >> frame_;
  if (frame_.empty()) {
    ROS_ERROR_STREAM("Failed to capture image!");
    ros::shutdown();
    exit(1);
  }
  msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();
  msg_->header.stamp = ros::Time::now();
  return msg_;
}

const CameraParams& Camera::GetParams() const {
  return params_;
}

}  // namespace opencv_camera
