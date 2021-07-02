#include "voxelgrid_filtering/voxelgrid_filtering.hpp"

namespace voxelgrid_filtering {
VoxelGridFilterNode::VoxelGridFilterNode(const rclcpp::NodeOptions& options)
    : Node("voxelgrid_filtering", options) {
  RCLCPP_INFO(this->get_logger(), "voxelgrid_filtering node has been created.");
}
}  // namespace voxelgrid_filtering

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(voxelgrid_filtering::VoxelGridFilterNode)