#include "pointcloud_fusion/pointcloud_fusion.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}