#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace voxelgrid_filtering {
class VoxelGridFilterNode : public rclcpp::Node {
 public:
  VoxelGridFilterNode(const rclcpp::NodeOptions& options);

 private:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  // subscriber callback function
  void cloudCallback(const PointCloudMsg::SharedPtr msg);

  // subscriber for original pointcloud
  rclcpp::Subscription<PointCloudMsg>::SharedPtr cloud_subscriber_;

  // publisher for downsampled pointcloud
  rclcpp::Publisher<PointCloudMsg>::SharedPtr downsampled_cloud_publisher_;

  // voxelgrid resolution
  float voxel_leaf_size_x_;
  float voxel_leaf_size_y_;
  float voxel_leaf_size_z_;
};
}  // namespace voxelgrid_filtering
