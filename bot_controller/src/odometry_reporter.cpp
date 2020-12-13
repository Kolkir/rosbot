#include "odometry_reporter.h"

#include <nav_msgs/Odometry.h>

OdometryReporter::OdometryReporter(ros::NodeHandle& node_handler,
                                   const BotParams& params) {
  last_time = ros::Time::now();
  odometry_publisher =
      node_handler.advertise<nav_msgs::Odometry>("bot_controller/odom", 50);
  dist_per_revolution = params.wheel_radius * 2.0 * M_PI;
  wheels_base = params.wheels_base;
}

ros::Time OdometryReporter::AddMeasurements(double right_revolutions,
                                            double left_revolutions) {
  auto right_dist = right_revolutions * dist_per_revolution;
  auto left_dist = left_revolutions * dist_per_revolution;

  auto current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();

  auto right_velocity = right_dist / dt;
  auto left_velocity = left_dist / dt;

  angular_velocity = (right_velocity - left_velocity) / wheels_base;
  linear_velocity = (left_velocity + right_velocity) / 2.0;

  // for defferential drive robot vy = 0

  double delta_x = linear_velocity * cos(th) * dt;
  double delta_y = linear_velocity * sin(th) * dt;
  double delta_th = angular_velocity * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  odometry_quaternion = tf::createQuaternionMsgFromYaw(th);

  last_time = current_time;
  return current_time;
}

void OdometryReporter::Report(ros::Time current_time) {
  ReportTransform(current_time);
  ReportOdometry();
}

void OdometryReporter::ReportTransform(ros::Time current_time) {
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odometry_quaternion;
  // ROS_INFO_STREAM("Transform:\n" << odom_trans);

  odometry_broadcaster.sendTransform(odom_trans);
}

void OdometryReporter::ReportOdometry() {
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odometry_quaternion;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = linear_velocity;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = angular_velocity;
  // ROS_INFO_STREAM("Odometry:\n" << odom);

  odometry_publisher.publish(odom);
}
