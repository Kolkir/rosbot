#ifndef POINTCLOUDMAPPER_H
#define POINTCLOUDMAPPER_H

#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "node_params.h"

namespace pcl_map {

using PclPointCloud = pcl::PointCloud<pcl::PointXYZ>;

class PointCloudMapper {
 public:
  PointCloudMapper(ros::Publisher& map_pub,
                   ros::Publisher& cloud_pub,
                   const Params& params)
      : map_pub_(map_pub), cloud_pub_(cloud_pub), params_(params) {}
  PointCloudMapper(const PointCloudMapper&) = delete;
  PointCloudMapper& operator=(const PointCloudMapper&) = delete;

  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

 private:
  void RemoveOutlierPoints(PclPointCloud::Ptr point_could);

  void RemoveGroundPoints(PclPointCloud::Ptr point_could);

  void ProjectPclToZPlane(PclPointCloud::Ptr point_could);

  nav_msgs::OccupancyGridPtr GenerateOccupancyGrid(
      PclPointCloud::ConstPtr point_could);

  void PopulateOccupancyGrid(PclPointCloud::ConstPtr point_could,
                             std::vector<signed char>& grid,
                             double x_min,
                             double y_min,
                             int width);

 private:
  ros::Publisher& map_pub_;
  ros::Publisher& cloud_pub_;
  const Params& params_;
  PclPointCloud::Ptr point_cloud_filtered_ =
      boost::make_shared<PclPointCloud>();
  std::vector<signed char> occupancy_grid_;
};

}  // namespace pcl_map

#endif  // POINTCLOUDMAPPER_H
