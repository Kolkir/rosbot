#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

namespace pcl_map {

class OccupancyGrid {
 public:
  using PclPointCloud = pcl::PointCloud<pcl::PointXYZ>;

  OccupancyGrid(double grid_resolution);
  void PopulateOccupancyGrid(PclPointCloud::ConstPtr point_could);

  int GetWidth() const;
  int GetHeight() const;
  PclPointCloud::value_type GetOrigin() const;
  void GetData(std::vector<signed char>& out) const;

 private:
  void ResizeStatic(const PclPointCloud::value_type& min,
                    const PclPointCloud::value_type& max);

  void ResizeSnapshot(const PclPointCloud::value_type& min,
                      const PclPointCloud::value_type& max);

 private:
  double grid_resolution_;
  cv::Mat data_;
  cv::Mat filtered_data_;
  PclPointCloud::value_type min_point_;
  PclPointCloud::value_type max_point_;
  int filter_block_size_ = 7;
  int morph_kern_size_ = 3;
  cv::Mat morph_element_;
  void UpdatePoints();
};

}  // namespace pcl_map
#endif  // OCCUPANCYGRID_H
