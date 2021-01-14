#include "orb_slam2_odom.h"

namespace orb_slam2_odom {
void ORBSlam2Odom::PoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  auto cur_time = ros::Time::now();
  tf::Pose cur_pose;
  tf::poseMsgToTF(msg->pose, cur_pose);
  if (!last_msg_time_.isZero()) {
    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = cur_time;
    odom_trans.header.frame_id = params_.frame_id;
    odom_trans.child_frame_id = params_.child_frame_id;

    odom_trans.transform.translation.x = cur_pose.getOrigin().getX();
    odom_trans.transform.translation.y = cur_pose.getOrigin().getY();
    odom_trans.transform.translation.z = cur_pose.getOrigin().getZ();
    odom_trans.transform.rotation.x = cur_pose.getRotation().getX();
    odom_trans.transform.rotation.y = cur_pose.getRotation().getY();
    odom_trans.transform.rotation.z = cur_pose.getRotation().getZ();
    odom_trans.transform.rotation.w = cur_pose.getRotation().getW();

    // send the transform
    odom_tf_broadcaster_.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = cur_time;
    odom.header.frame_id = params_.frame_id;
    // set the position
    odom.pose.pose.position.x = cur_pose.getOrigin().getX();
    odom.pose.pose.position.y = cur_pose.getOrigin().getY();
    odom.pose.pose.position.z = cur_pose.getOrigin().getZ();
    odom.pose.pose.orientation.x = cur_pose.getRotation().getX();
    odom.pose.pose.orientation.y = cur_pose.getRotation().getY();
    odom.pose.pose.orientation.z = cur_pose.getRotation().getZ();
    odom.pose.pose.orientation.w = cur_pose.getRotation().getW();

    // set the velocity
    auto time_diff = (cur_time - last_msg_time_).toSec();
    auto orig_diff = cur_pose.getOrigin() - last_pose_.getOrigin();

    odom.child_frame_id = params_.child_frame_id;
    odom.twist.twist.linear.x = orig_diff.getX() / time_diff;
    odom.twist.twist.linear.y = orig_diff.getY() / time_diff;
    odom.twist.twist.linear.z = orig_diff.getZ() / time_diff;

    tfScalar cur_yaw = 0, cur_pitch = 0, cur_roll = 0;
    cur_pose.getBasis().getEulerYPR(cur_yaw, cur_pitch, cur_roll);

    tfScalar last_yaw = 0, last_pitch = 0, last_roll = 0;
    last_pose_.getBasis().getEulerYPR(last_yaw, last_pitch, last_roll);

    tfScalar pitch_rate = (cur_pitch - last_pitch) / time_diff;
    tfScalar roll_rate = (cur_roll - last_roll) / time_diff;
    tfScalar yaw_rate = (cur_yaw - last_yaw) / time_diff;

    tfScalar wx = roll_rate + 0 - yaw_rate * sinf(cur_pitch);
    tfScalar wy = 0 + pitch_rate * cosf(cur_roll) +
                  yaw_rate * sinf(cur_roll) * cosf(cur_pitch);
    tfScalar wz = 0 - pitch_rate * sinf(cur_roll) +
                  yaw_rate * cosf(cur_roll) * cosf(cur_pitch);

    odom.twist.twist.angular.x = wx;
    odom.twist.twist.angular.y = wy;
    odom.twist.twist.angular.z = wz;

    // publish the message
    odom_pub_.publish(odom);

    // ROS_INFO_STREAM("ORB SLAM2 Odometry:\n" << odom);
  }
  last_msg_time_ = cur_time;
  last_pose_ = cur_pose;
}
}  // namespace orb_slam2_odom
