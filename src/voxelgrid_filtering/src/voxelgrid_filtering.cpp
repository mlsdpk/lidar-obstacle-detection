#include "voxelgrid_filtering/voxelgrid_filtering.hpp"

VoxelGridFilterNode::VoxelGridFilterNode() : Node("voxelgrid_filtering") {
  RCLCPP_INFO(this->get_logger(), "voxelgrid_filtering node has been created.");
}