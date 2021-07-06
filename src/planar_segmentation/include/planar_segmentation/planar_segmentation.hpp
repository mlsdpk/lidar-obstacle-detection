#pragma once

#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <random>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace planar_segmentation {
class PlanarSegmentationNode : public rclcpp::Node {
 public:
  PlanarSegmentationNode(const rclcpp::NodeOptions &options);

 private:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  // subscriber callback function
  void cloudCallback(const PointCloudMsg::SharedPtr msg);

  void ransac3D(const PointCloudMsg &in_cloud, PointCloudMsg &out_cloud,
                int max_iterations, float distance_tol);

  rclcpp::Subscription<PointCloudMsg>::SharedPtr cloud_subscriber_;

  rclcpp::Publisher<PointCloudMsg>::SharedPtr ground_plane_cloud_publisher_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr obst_plane_cloud_publisher_;

  int max_iters_;
  float distance_threshold_;
};
}  // namespace planar_segmentation