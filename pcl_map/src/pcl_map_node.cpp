#include <ros/ros.h>
#include "node_params.h"
#include "pointcloud_mapper.h"

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv) {
  using namespace pcl_map;
  ros::init(argc, argv, "pcl_map");
  ros::NodeHandle nh;

  Params params;
  params.Load(nh);
  params.Print();

  uint32_t queue_size = 1;
  ros::Publisher map_pub =
      nh.advertise<nav_msgs::OccupancyGrid>(params.map_out, queue_size);

  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>(params.cloud_out, queue_size);

  PointCloudMapper pcl_mapper(map_pub, cloud_pub, params);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
      params.cloud_in, queue_size, &PointCloudMapper::PointCloudCallback,
      &pcl_mapper);
  ros::spin();
}
