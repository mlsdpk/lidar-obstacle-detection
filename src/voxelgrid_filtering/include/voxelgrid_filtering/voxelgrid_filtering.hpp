#pragma once

#include "rclcpp/rclcpp.hpp"

namespace voxelgrid_filtering {
class VoxelGridFilterNode : public rclcpp::Node {
 public:
  VoxelGridFilterNode(const rclcpp::NodeOptions& options);
};
}  // namespace voxelgrid_filtering
