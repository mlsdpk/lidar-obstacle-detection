#include "rclcpp/rclcpp.hpp"
#include "voxelgrid_filtering/voxelgrid_filtering.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelGridFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}