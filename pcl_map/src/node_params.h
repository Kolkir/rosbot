#ifndef NODE_PARAMS_H
#define NODE_PARAMS_H

#include <ros/ros.h>
#include <string>

namespace pcl_map {

struct Params {
  std::string cloud_in = "cloud_in";
  std::string cloud_out = "cloud_out";
  std::string map_out = "map_out";
  double ground_start_z = 0.0;
  double ground_end_z = 1.0;
  double outliers_mean_k = 25.0;
  double outliers_stddev = 0.1;
  std::string frame_id = "map";
  double grid_resolution = 0.05;

  void Load(ros::NodeHandle& nh) {
    nh.param<std::string>("/pcl_map/frame_id", frame_id, frame_id);
    nh.param<std::string>("/pcl_map/cloud_in", cloud_in, cloud_in);
    nh.param<std::string>("/pcl_map/cloud_out", cloud_out, cloud_out);
    nh.param<std::string>("/pcl_map/map_out", map_out, map_out);
    nh.param("/pcl_map/ground_start_z", ground_start_z, ground_start_z);
    nh.param("/pcl_map/ground_end_z", ground_end_z, ground_end_z);
    nh.param("/pcl_map/outliers_mean_k", outliers_mean_k, outliers_mean_k);
    nh.param("/pcl_map/outliers_stddev", outliers_stddev, outliers_stddev);
    nh.param("/pcl_map/grig_resolution", grid_resolution, grid_resolution);
  }

  void Print() const {
    ROS_INFO_STREAM(
        "pcl_map params:\n"
        "frame_id : "
        << frame_id << "\n"
        << "cloud_in : " << cloud_in << "\n"
        << "cloud_out : " << cloud_out << "\n"
        << "map_out : " << map_out
        << "\n"
           "ground_start_z : "
        << ground_start_z
        << "\n"
           "ground_end_z : "
        << ground_end_z
        << "\n"
           "outliers_mean_k : "
        << outliers_mean_k
        << "\n"
           "outliers_stddev : "
        << outliers_stddev
        << "\n"
           "grig_resolution : "
        << grid_resolution << "\n");
  }
};
}  // namespace pcl_map
#endif  // NODE_PARAMS_H
