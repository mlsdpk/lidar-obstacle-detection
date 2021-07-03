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
  // ros pcl_conversions cannot use here since its only using older pcl version
  // which does not suppot c++11 smart pointers

  pcl::PCLPointCloud2::Ptr input_cloud(new pcl::PCLPointCloud2);

  // // header
  input_cloud->header.stamp =
      msg->header.stamp.nanosec / 1000ull;  // Convert from ns to us
  input_cloud->header.seq = 0;
  input_cloud->header.frame_id = msg->header.frame_id;

  input_cloud->height = msg->height;
  input_cloud->width = msg->width;

  // // fields
  input_cloud->fields.resize(msg->fields.size());
  std::vector<sensor_msgs::msg::PointField>::const_iterator it =
      msg->fields.begin();
  int i = 0;
  for (; it != msg->fields.end(); ++it, ++i) {
    input_cloud->fields[i].name = it->name;
    input_cloud->fields[i].offset = it->offset;
    input_cloud->fields[i].datatype = it->datatype;
    input_cloud->fields[i].count = it->count;
  }
  input_cloud->is_bigendian = msg->is_bigendian;
  input_cloud->point_step = msg->point_step;
  input_cloud->row_step = msg->row_step;
  input_cloud->is_dense = msg->is_dense;

  // copy pointcloud data
  input_cloud->data = msg->data;

  // create voxel grid object
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;

  vg.setInputCloud(input_cloud);
  vg.setLeafSize(voxel_leaf_size_x_, voxel_leaf_size_y_, voxel_leaf_size_z_);

  pcl::PCLPointCloud2::Ptr filtered_cloud(new pcl::PCLPointCloud2);
  vg.filter(*filtered_cloud);

  // convert back to ROS datatype
  PointCloudMsg downsampled_cloud_msg;

  // header
  downsampled_cloud_msg.header.stamp =
      rclcpp::Time(filtered_cloud->header.stamp * 1000ull);
  downsampled_cloud_msg.header.frame_id = filtered_cloud->header.frame_id;

  downsampled_cloud_msg.height = filtered_cloud->height;
  downsampled_cloud_msg.width = filtered_cloud->width;

  // fields
  downsampled_cloud_msg.fields.resize(filtered_cloud->fields.size());

  std::vector<pcl::PCLPointField>::const_iterator it1 =
      filtered_cloud->fields.begin();
  int j = 0;
  for (; it1 != filtered_cloud->fields.end(); ++it1, ++j) {
    downsampled_cloud_msg.fields[j].name = it1->name;
    downsampled_cloud_msg.fields[j].offset = it1->offset;
    downsampled_cloud_msg.fields[j].datatype = it1->datatype;
    downsampled_cloud_msg.fields[j].count = it1->count;
  }

  downsampled_cloud_msg.is_bigendian = filtered_cloud->is_bigendian;
  downsampled_cloud_msg.point_step = filtered_cloud->point_step;
  downsampled_cloud_msg.row_step = filtered_cloud->row_step;
  downsampled_cloud_msg.is_dense = filtered_cloud->is_dense;

  // moving instead of copying (swaping vectors in constant time)
  downsampled_cloud_msg.data.swap(filtered_cloud->data);

  // publish
  downsampled_cloud_publisher_->publish(downsampled_cloud_msg);
}

}  // namespace voxelgrid_filtering

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(voxelgrid_filtering::VoxelGridFilterNode)