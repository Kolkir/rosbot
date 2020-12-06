#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

struct CameraParams {
  int camera_index = 0;
  double frame_width = 0.0;
  double frame_height = 0.0;
  double frame_rate = 0.0;
};

CameraParams ReadCameraParams(ros::NodeHandle& node_handler) {
  CameraParams params;

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "opencv_camera");
  ros::NodeHandle node_handler;

  CameraParams params = ReadCameraParams(node_handler);

  cv::VideoCapture capture(params.camera_index, cv::CAP_V4L2);
  if (!capture.isOpened()) {
    ROS_ERROR_STREAM("Failed to open camera with index " << params.camera_index
                                                         << "!");
    ros::shutdown();
    exit(1);
  }

  if (!capture.set(cv::CAP_PROP_FRAME_WIDTH, params.frame_width)) {
    ROS_ERROR_STREAM("Failed to set frame with " << params.frame_width);
  }

  if (!capture.set(cv::CAP_PROP_FRAME_HEIGHT, params.frame_height)) {
    ROS_ERROR_STREAM("Failed to set frame height " << params.frame_height);
  }

  if (!capture.set(cv::CAP_PROP_FPS, params.frame_rate)) {
    ROS_ERROR_STREAM("Failed to set frame rate " << params.frame_rate);
  }

  cv::Mat frame;
  capture >> frame;
  if (frame.empty()) {
    ROS_ERROR_STREAM("Failed to capture image!");
    ros::shutdown();
    exit(1);
  }

  ROS_WARN_STREAM("Frame width = " << frame.cols << " height = " << frame.rows);

  // Image_transport is responsible for publishing and subscribing to Images
  image_transport::ImageTransport it(node_handler);

  // Publish to the /camera topic
  image_transport::Publisher pub_frame = it.advertise("opencv_camera", 1);

  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(params.frame_rate);

  while (node_handler.ok()) {
    capture >> frame;

    if (frame.empty()) {
      ROS_ERROR_STREAM("Failed to capture image!");
      ros::shutdown();
      exit(1);
    }

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub_frame.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Shutdown the camera
  capture.release();
}
