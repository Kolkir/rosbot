#include "occupancy_grid.h"
#include <pcl/common/common.h>
#include <ros/ros.h>

namespace pcl_map {

static auto new_value = (uchar)(200);
static auto old_value = (uchar)(100);

OccupancyGrid::OccupancyGrid(double grid_resolution)
    : grid_resolution_(grid_resolution) {
  data_ = cv::Mat(1, 1, CV_8UC1);
  cv::Mat element = cv::getStructuringElement(
      cv::MORPH_CROSS,
      cv::Size(2 * morph_kern_size_ + 1, 2 * morph_kern_size_ + 1),
      cv::Point(morph_kern_size_, morph_kern_size_));
}

void OccupancyGrid::ResizeStatic(const PclPointCloud::value_type& min,
                                 const PclPointCloud::value_type& max) {
  // don't make the grid smaller because it can lead to errors in other nodes
  PclPointCloud::value_type new_min_point(std::min(min_point_.x, min.x),
                                          std::min(min_point_.y, min.y),
                                          std::min(min_point_.z, min.z));

  PclPointCloud::value_type new_max_point(std::max(max_point_.x, max.x),
                                          std::max(max_point_.y, max.y),
                                          std::max(max_point_.z, max.z));

  auto top = std::max(std::abs(min_point_.y), std::abs(new_min_point.y)) -
             std::min(std::abs(min_point_.y), std::abs(new_min_point.y));
  auto left = std::max(std::abs(min_point_.x), std::abs(new_min_point.x)) -
              std::min(std::abs(min_point_.x), std::abs(new_min_point.x));

  auto bottom = std::max(std::abs(max_point_.y), std::abs(new_max_point.y)) -
                std::min(std::abs(max_point_.y), std::abs(new_max_point.y));
  auto right = std::max(std::abs(max_point_.x), std::abs(new_max_point.x)) -
               std::min(std::abs(max_point_.x), std::abs(new_max_point.x));

  top = static_cast<int>(top / grid_resolution_);
  left = static_cast<int>(left / grid_resolution_);
  bottom = static_cast<int>(bottom / grid_resolution_);
  right = static_cast<int>(right / grid_resolution_);

  if (top > 0 || left > 0 || bottom > 0 || right > 0) {
    ROS_INFO_STREAM("Resize top " << top << " left " << left << " bottom "
                                  << bottom << " right " << right);
    cv::copyMakeBorder(data_, data_, top, bottom, left, right,
                       cv::BORDER_CONSTANT, cv::Scalar(0));

    ROS_INFO_STREAM("Occupancy grid size w = " << data_.cols
                                               << " h = " << data_.rows);
  }

  min_point_ = new_min_point;
  max_point_ = new_max_point;
}

void OccupancyGrid::ResizeSnapshot(const PclPointCloud::value_type& min,
                                   const PclPointCloud::value_type& max) {
  min_point_ = min;
  max_point_ = max;
  int width =
      static_cast<int>((max_point_.x - min_point_.x) / grid_resolution_) + 1;
  int height =
      static_cast<int>((max_point_.y - min_point_.y) / grid_resolution_) + 1;

  // don't make the grid smaller because it can lead to errors in other nodes
  width = std::max(data_.cols, width);
  height = std::max(data_.rows, height);

  data_ = cv::Mat::zeros(height, width, CV_8UC1);
}

void OccupancyGrid::UpdatePoints() {
  auto block_shift = filter_block_size_ / 2;
  double min = 0, max = 0;
  cv::Mat roi;
  int points_removed = 0;
  for (int y = 0; y < data_.rows; ++y) {
    for (int x = 0; x < data_.cols; ++x) {
      // using sliding window approach
      auto x_min = std::max(0, x - block_shift);
      auto x_max = std::min(data_.cols, x + block_shift + 1);
      auto y_min = std::max(0, y - block_shift);
      auto y_max = std::min(data_.rows, y + block_shift + 1);

      roi = data_(cv::Range(y_min, y_max), cv::Range(x_min, x_max));

      cv::minMaxIdx(roi, &min, &max);
      auto& value = data_.at<uchar>(y, x, 0);
      if (max >= new_value && value == old_value) {
        // clear point
        value = 0;
        ++points_removed;
      }
    }
  }

  if (points_removed != 0) {
    ROS_INFO_STREAM("Occupancy grid: " << points_removed
                                       << " old points removed");
  }

  // set uniform values
  for (int y = 0; y < data_.rows; ++y) {
    for (int x = 0; x < data_.cols; ++x) {
      auto& value = data_.at<uchar>(y, x, 0);
      if (value > old_value) {
        value = old_value;
      }
    }
  }
}

void OccupancyGrid::PopulateOccupancyGrid(PclPointCloud::ConstPtr point_could) {
  PclPointCloud::value_type min_point, max_point;
  pcl::getMinMax3D(*point_could, min_point, max_point);
  ResizeSnapshot(min_point, max_point);

  // mark new points
  for (size_t i = 0; i < point_could->size(); i++) {
    auto x = point_could->points[i].x;
    auto y = point_could->points[i].y;

    int x_cell = static_cast<int>((x - min_point_.x) / grid_resolution_);
    int y_cell = static_cast<int>((y - min_point_.y) / grid_resolution_);

    data_.at<uchar>(y_cell, x_cell, 0) = old_value;  // new_value
  }

  //  ROS_INFO_STREAM("Occupancy grid: " << point_could->size()
  //                                     << " new points added");

  // remove old points only if there are new points as neighbors
  // UpdatePoints();

  // clear noise
  cv::erode(data_, filtered_data_, morph_element_);
  cv::dilate(data_, filtered_data_, morph_element_);
  // cv::morphologyEx(data_, filtered_data_, cv::MORPH_CLOSE, morph_element_);
  // cv::morphologyEx(filtered_data_, filtered_data_, cv::MORPH_OPEN,
  //                 morph_element_);
}  // namespace pcl_map

int OccupancyGrid::GetWidth() const {
  return data_.cols;
}
int OccupancyGrid::GetHeight() const {
  return data_.rows;
}
OccupancyGrid::PclPointCloud::value_type OccupancyGrid::GetOrigin() const {
  return min_point_;
}
void OccupancyGrid::GetData(std::vector<signed char>& out) const {
  out.resize(filtered_data_.total());
  auto ptr = filtered_data_.ptr<signed char>(0);
  out.assign(ptr, ptr + filtered_data_.total());
}

}  // namespace pcl_map
