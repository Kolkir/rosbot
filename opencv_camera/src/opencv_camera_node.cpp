#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/assign/list_of.hpp>
#include <opencv2/highgui/highgui.hpp>

struct CameraParams {
  int camera_index = 0;
  std::string name;
  std::string info_file_url;
  double frame_width = 0.0;
  double frame_height = 0.0;
  double frame_rate = 0.0;
};

CameraParams ReadCameraParams(ros::NodeHandle& node_handler) {
  CameraParams params;

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

sensor_msgs::CameraInfo MakeDefaultCameraInfo(sensor_msgs::ImagePtr img) {
  sensor_msgs::CameraInfo cam_info_msg;
  cam_info_msg.header.frame_id = img->header.frame_id;
  // Fill image size
  cam_info_msg.height = img->height;
  cam_info_msg.width = img->width;
  // Add the most common distortion model as sensor_msgs/CameraInfo says
  cam_info_msg.distortion_model = "plumb_bob";
  // Don't let distorsion matrix be empty
  cam_info_msg.D.resize(5, 0.0);
  // Give a reasonable default intrinsic camera matrix
  cam_info_msg.K =
      boost::assign::list_of(img->width / 2.0)(0.0)(img->width / 2.0)(0.0)(
          img->height / 2.0)(img->height / 2.0)(0.0)(0.0)(1.0);
  // Give a reasonable default rectification matrix
  cam_info_msg.R =
      boost::assign::list_of(1.0)(0.0)(0.0)(0.0)(1.0)(0.0)(0.0)(0.0)(1.0);
  // Give a reasonable default projection matrix
  cam_info_msg.P =
      boost::assign::list_of(img->width / 2.0)(0.0)(img->width / 2.0)(0.0)(0.0)(
          img->height / 2.0)(img->height / 2.0)(0.0)(0.0)(0.0)(1.0)(0.0);
  return cam_info_msg;
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

  ros::WallRate loop_rate(ros::Duration(1.0 / params.frame_rate));

  while (node_handler.ok()) {
    capture >> frame;

    if (frame.empty()) {
      ROS_ERROR_STREAM("Failed to capture image!");
      ros::shutdown();
      exit(1);
    }

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    // Create a default camera info if we didn't get a stored one on
    // initialization
    if (cam_info_msg.distortion_model == "") {
      ROS_WARN_STREAM(
          "No calibration file given, publishing a reasonable default camera "
          "info.");
      cam_info_msg = MakeDefaultCameraInfo(msg);
    }
    pub_frame.publish(*msg, cam_info_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Shutdown the camera
  capture.release();
}
