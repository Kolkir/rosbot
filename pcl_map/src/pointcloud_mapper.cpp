#include "pointcloud_mapper.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

namespace pcl_map {

using DataType = float;

void PointCloudMapper::PointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  auto point_could = boost::make_shared<PclPointCloud>();
  pcl::fromROSMsg(*msg, *point_could);

  // remove outliers
  RemoveOutlierPoints(point_could);

  RemoveGroundPoints(point_could);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*point_could, cloud_msg);
  cloud_pub_.publish(cloud_msg);

  ProjectPclToZPlane(point_could);

  auto out_msg = GenerateOccupancyGrid(point_could);

  map_pub_.publish(out_msg);
}

void PointCloudMapper::PopulateOccupancyGrid(
    PclPointCloud::ConstPtr point_could,
    std::vector<signed char>& grid,
    double x_min,
    double y_min,
    int width) {
  for (size_t i = 0; i < point_could->size(); i++) {
    auto x = point_could->points[i].x;
    auto y = point_could->points[i].y;

    int x_cell = static_cast<int>((x - x_min) / params_.grid_resolution);
    int y_cell = static_cast<int>((y - y_min) / params_.grid_resolution);
    grid[y_cell * width + x_cell] = char(100);
  }
}

nav_msgs::OccupancyGridPtr PointCloudMapper::GenerateOccupancyGrid(
    PclPointCloud::ConstPtr point_could) {
  PclPointCloud::value_type min_point, max_point;
  pcl::getMinMax3D(*point_could, min_point, max_point);

  int width =
      static_cast<int>((max_point.x - min_point.x) / params_.grid_resolution) +
      1;
  int height =
      static_cast<int>((max_point.y - min_point.y) / params_.grid_resolution) +
      1;

  // don't make the grid smaller because it can lead to errors in other nodes
  grid_width_ = std::max(grid_width_, width);
  grid_height_ = std::max(grid_height_, height);

  occupancy_grid_.resize(grid_width_ * grid_height_);
  std::fill(occupancy_grid_.begin(), occupancy_grid_.end(), 0);
  PopulateOccupancyGrid(point_could, occupancy_grid_, min_point.x, min_point.y,
                        grid_width_);

  auto grid = boost::make_shared<nav_msgs::OccupancyGrid>();
  grid->header.seq = 1;
  grid->header.frame_id = params_.frame_id;
  grid->info.origin.position.z = 0;
  grid->info.origin.orientation.w = 1;
  grid->info.origin.orientation.x = 0;
  grid->info.origin.orientation.y = 0;
  grid->info.origin.orientation.z = 0;
  auto cur_time = ros::Time::now();
  grid->header.stamp.sec = cur_time.sec;
  grid->header.stamp.nsec = cur_time.nsec;
  grid->info.map_load_time = cur_time;
  grid->info.resolution = params_.grid_resolution;
  grid->info.width = grid_width_;
  grid->info.height = grid_height_;
  grid->info.origin.position.x = min_point.x;
  grid->info.origin.position.y = min_point.y;
  grid->data = occupancy_grid_;

  return grid;
}

void PointCloudMapper::RemoveOutlierPoints(PclPointCloud::Ptr point_could) {
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(point_could);
  sor.setMeanK(params_.outliers_mean_k);
  sor.setStddevMulThresh(params_.outliers_stddev);
  sor.filter(*point_cloud_filtered_);
  *point_could = *point_cloud_filtered_;
}

void PointCloudMapper::RemoveGroundPoints(PclPointCloud::Ptr point_could) {
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(point_could);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(params_.ground_start_z, params_.ground_end_z);
  pass.setFilterLimitsNegative(false);
  pass.filter(*point_cloud_filtered_);
  *point_could = *point_cloud_filtered_;
}

void PointCloudMapper::ProjectPclToZPlane(PclPointCloud::Ptr point_could) {
  // Create a set of planar coefficients with X=Y=0,Z=1
  auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
  coefficients->values.resize(4);
  coefficients->values[0] = 0.0;
  coefficients->values[1] = 0.0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0.0;

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(point_could);
  proj.setModelCoefficients(coefficients);
  proj.filter(*point_cloud_filtered_);
  *point_could = *point_cloud_filtered_;
}

}  // namespace pcl_map
