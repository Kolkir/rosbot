#ifndef NODE_PARAMS_H
#define NODE_PARAMS_H

#include <ros/ros.h>
#include <string>

namespace orb_slam2_odom {

struct Params {
  std::string frame_id = "odom";
  std::string child_frame_id = "base_link";
  std::string pose_in = "pose_in";
  std::string odom_topic = "odom_out";

  void Load(ros::NodeHandle& nh) {
    nh.param<std::string>("/orb_slam2_odom/frame_id", frame_id, frame_id);
    nh.param<std::string>("/orb_slam2_odom/child_frame_id", child_frame_id,
                          child_frame_id);
    nh.param<std::string>("/orb_slam2_odom/pose_in", pose_in, pose_in);
    nh.param<std::string>("/orb_slam2_odom/odom_topic", odom_topic, odom_topic);
  }

  void Print() const {
    ROS_INFO_STREAM("orb_slam2_odom params:\n"
                    << "frame_id : " << frame_id << "\n"
                    << "child_frame_id : " << child_frame_id << "\n"
                    << "pose_in : " << pose_in << "\n"
                    << "odom_out : " << odom_topic << "\n");
  }
};
}  // namespace orb_slam2_odom
#endif  // NODE_PARAMS_H
