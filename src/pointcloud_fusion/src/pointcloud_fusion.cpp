#include "pointcloud_fusion/pointcloud_fusion.hpp"

namespace pointcloud_fusion {
PointCloudFusionNode::PointCloudFusionNode(const rclcpp::NodeOptions &options)
    : Node("pointcloud_fusion", options),
      fused_point_cloud_publisher_{
          create_publisher<PointCloudMsg>("output", 10)},
      fused_frame_name_{
          declare_parameter("fused_frame_name").get<std::string>()},
      fused_point_cloud_max_capacity_{static_cast<uint32_t>(
          declare_parameter("fused_point_cloud_max_capacity").get<int>())} {
  RCLCPP_INFO(this->get_logger(), "pointcloud_fusion node has been created.");

  /////////////////////////////////////////////////////////////////
  // find static tfs from lidar_front and lidar_rear to base_link
  // lookup from /tf or /static_tf topics

  // tf2 buffer
  tf2_ros::Buffer tf2_buffer(this->get_clock());
  // tf2 listener
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  geometry_msgs::msg::Transform tx_front_lidar;
  geometry_msgs::msg::Transform tx_rear_lidar;
  while (rclcpp::ok()) {
    try {
      RCLCPP_INFO(get_logger(), "Looking up the transform.");
      tx_front_lidar = tf2_buffer
                           .lookupTransform(fused_frame_name_, "lidar_front",
                                            tf2::TimePointZero)
                           .transform;
      tx_rear_lidar = tf2_buffer
                          .lookupTransform(fused_frame_name_, "lidar_rear",
                                           tf2::TimePointZero)
                          .transform;
      break;
    } catch (const std::exception &transform_exception) {
      RCLCPP_INFO(get_logger(),
                  "No transform was available. Retrying after 100 ms.");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
  }

  RCLCPP_INFO(get_logger(), "Found transform.");

  // convert each geometry_msgs::msg::Transform type of lidar tfs
  // to eigen affine3f
  convertToAffine3f(tx_front_lidar, affine3f_front_lidar_);
  convertToAffine3f(tx_rear_lidar, affine3f_rear_lidar_);
  RCLCPP_INFO(get_logger(), "Transformation completed.");
  /////////////////////////////////////////////////////////////////

  // initialize fused point cloud message
  fused_point_cloud_.height = 1U;
  fused_point_cloud_.is_bigendian = false;
  fused_point_cloud_.is_dense = false;
  fused_point_cloud_.header.frame_id = fused_frame_name_;
  sensor_msgs::PointCloud2Modifier modifier(fused_point_cloud_);
  modifier.setPointCloud2Fields(
      4, "x", 1U, sensor_msgs::msg::PointField::FLOAT32, "y", 1U,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1U,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1U,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(fused_point_cloud_max_capacity_);

  // initialize front and rear lidar subscriber objects
  front_lidar_subscriber_ =
      std::make_unique<message_filters::Subscriber<PointCloudMsg>>(this,
                                                                   "input1");
  rear_lidar_subscriber_ =
      std::make_unique<message_filters::Subscriber<PointCloudMsg>>(this,
                                                                   "input2");

  // initialize message filter stuffs
  point_cloud_synchronizer_ =
      std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
          SyncPolicy(10), *front_lidar_subscriber_, *rear_lidar_subscriber_);

  point_cloud_synchronizer_->registerCallback(
      std::bind(&PointCloudFusionNode::pointCloudCallback, this,
                std::placeholders::_1, std::placeholders::_2));
}

void PointCloudFusionNode::convertToAffine3f(
    const geometry_msgs::msg::Transform &tf, Eigen::Affine3f &af) {
  Eigen::Quaternionf rotation{
      static_cast<float>(tf.rotation.w), static_cast<float>(tf.rotation.x),
      static_cast<float>(tf.rotation.y), static_cast<float>(tf.rotation.z)};

  // commented out for now
  // need to add later (reference from autoware)
  // if (!comp::rel_eq(rotation.norm(), 1.0f,
  //                   std::numeric_limits<float>::epsilon();)) {
  //   throw std::domain_error("StaticTransformer: quaternion is not
  //   normalized");
  // }

  af.setIdentity();
  af.linear() = rotation.toRotationMatrix();
  af.translation() = Eigen::Vector3f{static_cast<float>(tf.translation.x),
                                     static_cast<float>(tf.translation.y),
                                     static_cast<float>(tf.translation.z)};
}

void PointCloudFusionNode::pointCloudCallback(
    const PointCloudMsg::ConstSharedPtr &msg1,
    const PointCloudMsg::ConstSharedPtr &msg2) {
  // reset fused point cloud message
  sensor_msgs::PointCloud2Modifier modifier(fused_point_cloud_);
  modifier.clear();
  modifier.resize(fused_point_cloud_max_capacity_);

  // get latest timestamp
  auto latest_timestamp = msg1->header.stamp;
  if (std::chrono::nanoseconds(latest_timestamp.nanosec) <
      std::chrono::nanoseconds(msg2->header.stamp.nanosec)) {
    latest_timestamp = msg2->header.stamp;
  }

  // get total size of point clouds
  // auto total_size = msg1->width + msg2->width;

  // TODO: check to make sure total size is not greater than
  //       fused_point_cloud_ msg size

  uint32_t point_cloud_idx = 0;
  concatenatePointCloud(*msg1, fused_point_cloud_, point_cloud_idx,
                        affine3f_front_lidar_);
  concatenatePointCloud(*msg2, fused_point_cloud_, point_cloud_idx,
                        affine3f_rear_lidar_);

  // resize and publish
  modifier.resize(point_cloud_idx);

  // RCLCPP_INFO(this->get_logger(), "here %d.", point_cloud_idx);
  fused_point_cloud_.header.stamp = latest_timestamp;
  fused_point_cloud_publisher_->publish(fused_point_cloud_);
}

void PointCloudFusionNode::concatenatePointCloud(
    const PointCloudMsg &pc_in, PointCloudMsg &pc_out, uint32_t &concat_idx,
    const Eigen::Affine3f &affine_tf) {
  // TODO: add error handlings

  // TODO: float needs to be 32-bit
  // assert if it is not true
  // get const iterator from input msg
  sensor_msgs::PointCloud2ConstIterator<float> x_it_in(pc_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it_in(pc_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it_in(pc_in, "z");
  sensor_msgs::PointCloud2ConstIterator<float> intensity_it_in(pc_in,
                                                               "intensity");

  // get iterator from output msg
  sensor_msgs::PointCloud2Iterator<float> x_it_out(pc_out, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it_out(pc_out, "y");
  sensor_msgs::PointCloud2Iterator<float> z_it_out(pc_out, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity_it_out(pc_out, "intensity");

  const int idx = static_cast<int>(concat_idx);
  x_it_out += idx;
  y_it_out += idx;
  z_it_out += idx;
  intensity_it_out += idx;

  while (x_it_in != x_it_in.end() && y_it_in != y_it_in.end() &&
         z_it_in != z_it_in.end() && intensity_it_in != intensity_it_in.end()) {
    // input point
    auto x_in = *x_it_in;
    auto y_in = *y_it_in;
    auto z_in = *z_it_in;
    auto intensity_in = *intensity_it_in;

    // apply static transform from input point to baselink
    Eigen::Vector3f out_mat = affine_tf * Eigen::Vector3f{x_in, y_in, z_in};
    x_in = out_mat[0];
    y_in = out_mat[1];
    z_in = out_mat[2];

    // add input to output msg if its idx is not at the end of iter
    if (x_it_out != x_it_out.end() && y_it_out != y_it_out.end() &&
        z_it_out != z_it_out.end() &&
        intensity_it_out != intensity_it_out.end()) {
      // add input point
      *x_it_out = x_in;
      *y_it_out = y_in;
      *z_it_out = z_in;
      *intensity_it_out = intensity_in;

      // increment the index to keep track of the pointcloud's size
      ++concat_idx;
    }

    ++x_it_in;
    ++y_it_in;
    ++z_it_in;
    ++intensity_it_in;

    ++x_it_out;
    ++y_it_out;
    ++z_it_out;
    ++intensity_it_out;
  }
}
}  // namespace pointcloud_fusion

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_fusion::PointCloudFusionNode)
