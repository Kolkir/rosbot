#include <ros/ros.h>
#include "node_params.h"
#include "orb_slam2_odom.h"

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv) {
  using namespace orb_slam2_odom;
  ros::init(argc, argv, "orb_salm_2_odom");
  ros::NodeHandle nh;

  Params params;
  params.Load(nh);
  params.Print();

  uint32_t queue_size = 1;
  ros::Publisher odom_pub =
      nh.advertise<nav_msgs::Odometry>(params.odom_topic, queue_size);

  ORBSlam2Odom slam_odom(odom_pub, params);

  ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(
      params.pose_in, queue_size, &ORBSlam2Odom::PoseCallback, &slam_odom);
  ros::spin();
}
