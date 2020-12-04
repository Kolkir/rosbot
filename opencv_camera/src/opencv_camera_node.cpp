#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "opencv_camera");
  ros::NodeHandle node_handler;

  int camera_index = 0;
  if (!node_handler.param("/opencv_camera/camera_index", camera_index, 0)) {
    ROS_ERROR_STREAM("Using default camera index param");
  } else {
    ROS_WARN_STREAM("Camera index param = " << camera_index);
  }

  double frame_width = 0;
  if (!node_handler.param("/opencv_camera/frame_width", frame_width, 640.0)) {
    ROS_ERROR_STREAM("Using default frame width param");
  } else {
    ROS_WARN_STREAM("Frame width param = " << frame_width);
  }

  double frame_height = 0;
  if (!node_handler.param("/opencv_camera/frame_height", frame_height, 480.0)) {
    ROS_ERROR_STREAM("Using default frame height param");
  } else {
    ROS_WARN_STREAM("Frame height param = " << frame_height);
  }

  double frame_rate = 0.0;
  if (!node_handler.param("/opencv_camera/frame_rate", frame_rate, 15.0)) {
    ROS_ERROR_STREAM("Using default frame rate param");
  }

  cv::VideoCapture capture(camera_index, cv::CAP_V4L2);
  if (!capture.isOpened()) {
    ROS_ERROR_STREAM("Failed to open camera with index " << camera_index
                                                         << "!");
    ros::shutdown();
  }

  if (!capture.set(cv::CAP_PROP_FRAME_WIDTH, frame_width)) {
    ROS_ERROR_STREAM("Failed to set frame with " << frame_width);
  } else {
    ROS_WARN_STREAM("Frame width configured to "
                    << capture.get(cv::CAP_PROP_FRAME_WIDTH));
  }

  if (!capture.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height)) {
    ROS_ERROR_STREAM("Failed to set frame height " << frame_height);
  } else {
    ROS_WARN_STREAM("Frame height configured to "
                    << capture.get(cv::CAP_PROP_FRAME_HEIGHT));
  }

  if (!capture.set(cv::CAP_PROP_FPS, frame_rate)) {
    ROS_ERROR_STREAM("Failed to set frame rate " << frame_rate);
  } else {
    ROS_WARN_STREAM("Frame rate configured to "
                    << capture.get(cv::CAP_PROP_FPS));
  }

  // Image_transport is responsible for publishing and subscribing to Images
  image_transport::ImageTransport it(node_handler);

  // Publish to the /camera topic
  image_transport::Publisher pub_frame = it.advertise("opencv_camera", 1);

  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(frame_rate);

  while (node_handler.ok()) {
    // Load image
    capture >> frame;

    // Check if grabbed frame has content
    if (frame.empty()) {
      ROS_ERROR_STREAM("Failed to capture image!");
      ros::shutdown();
    }

    // Convert image from cv::Mat (OpenCV) type to sensor_msgs/Image (ROS) type
    // and publish
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub_frame.publish(msg);
    /*
    Cv_bridge can selectively convert color and depth information. In order to
    use the specified feature encoding, there is a centralized coding form:
      Mono8: CV_8UC1, grayscale image
      Mono16: CV_16UC1, 16-bit grayscale image
      Bgr8: CV_8UC3 with color information and the order of colors is BGR order
      Rgb8: CV_8UC3 with color information and the order of colors is RGB order
      Bgra8: CV_8UC4, BGR color image with alpha channel
      Rgba8: CV_8UC4, CV, RGB color image with alpha channel
    */

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Shutdown the camera
  capture.release();
}
