#include "voxelgrid_filtering/voxelgrid_filtering.hpp"

namespace voxelgrid_filtering {
VoxelGridFilterNode::VoxelGridFilterNode(const rclcpp::NodeOptions& options)
    : Node("voxelgrid_filtering", options),
      cloud_subscriber_{create_subscription<PointCloudMsg>(
          "input", 10,
          std::bind(&VoxelGridFilterNode::cloudCallback, this,
                    std::placeholders::_1))},
      downsampled_cloud_publisher_{
          create_publisher<PointCloudMsg>("output", 10)} {
  RCLCPP_INFO(this->get_logger(), "voxelgrid_filtering node has been created.");

  voxel_leaf_size_x_ =
      static_cast<float>(declare_parameter("voxel_leaf_size.x").get<float>());
  voxel_leaf_size_y_ =
      static_cast<float>(declare_parameter("voxel_leaf_size.y").get<float>());
  voxel_leaf_size_z_ =
      static_cast<float>(declare_parameter("voxel_leaf_size.z").get<float>());
}

void VoxelGridFilterNode::cloudCallback(const PointCloudMsg::SharedPtr msg) {
  // convert from rosmsg to pcl
  // create pcl pointXYZI object
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*msg, *input_cloud);

  // create voxel grid object
  pcl::VoxelGrid<pcl::PointXYZI> vg;

  vg.setInputCloud(input_cloud);
  vg.setLeafSize(voxel_leaf_size_x_, voxel_leaf_size_y_, voxel_leaf_size_z_);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  vg.filter(*filtered_cloud);

  // convert back to ROS datatype
  PointCloudMsg downsampled_cloud_msg;
  pcl::toROSMsg(*filtered_cloud, downsampled_cloud_msg);

  // publish
  downsampled_cloud_publisher_->publish(downsampled_cloud_msg);
}

}  // namespace voxelgrid_filtering

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(voxelgrid_filtering::VoxelGridFilterNode)