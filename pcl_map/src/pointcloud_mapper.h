#ifndef POINTCLOUDMAPPER_H
#define POINTCLOUDMAPPER_H

#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "node_params.h"
#include "occupancy_grid.h"

namespace pcl_map {

using PclPointCloud = pcl::PointCloud<pcl::PointXYZ>;

class PointCloudMapper {
 public:
  PointCloudMapper(ros::Publisher& map_pub,
                   ros::Publisher& cloud_pub,
                   const Params& params)
      : map_pub_(map_pub),
        cloud_pub_(cloud_pub),
        params_(params),
        occupancy_grid_(params.grid_resolution) {}
  PointCloudMapper(const PointCloudMapper&) = delete;
  PointCloudMapper& operator=(const PointCloudMapper&) = delete;

  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

 private:
  void RemoveOutlierPoints(PclPointCloud::Ptr point_could);

  void RemoveGroundPoints(PclPointCloud::Ptr point_could);

  void ProjectPclToZPlane(PclPointCloud::Ptr point_could);

  nav_msgs::OccupancyGridPtr GenerateOccupancyGrid(
      PclPointCloud::ConstPtr point_could);

 private:
  ros::Publisher& map_pub_;
  ros::Publisher& cloud_pub_;
  const Params& params_;
  PclPointCloud::Ptr point_cloud_filtered_ =
      boost::make_shared<PclPointCloud>();
  OccupancyGrid occupancy_grid_;
};

}  // namespace pcl_map

#endif  // POINTCLOUDMAPPER_H
