#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "opencv_camera");

  nodelet::Loader manager(true);
  nodelet::M_string remappings(ros::names::getRemappings());
  nodelet::V_string my_argv(argv + 1, argv + argc);

  manager.load(ros::this_node::getName(), "opencv_camera/VideoStream",
               remappings, my_argv);

  ros::spin();
}
