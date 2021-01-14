#ifndef ORBSLAM2ODOM_H
#define ORBSLAM2ODOM_H

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "node_params.h"

namespace orb_slam2_odom {
class ORBSlam2Odom {
 public:
  ORBSlam2Odom(ros::Publisher& odom_pub, const Params& params)
      : odom_pub_(odom_pub), params_(params) {}
  ORBSlam2Odom(const ORBSlam2Odom&) = delete;
  ORBSlam2Odom& operator=(const ORBSlam2Odom&) = delete;

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

 private:
  ros::Publisher& odom_pub_;
  const Params& params_;
  ros::Time last_msg_time_;
  tf::TransformBroadcaster odom_tf_broadcaster_;
  tf::Pose last_pose_;
};
}  // namespace orb_slam2_odom
#endif  // ORBSLAM2ODOM_H
