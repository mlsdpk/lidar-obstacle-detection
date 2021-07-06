#include "planar_segmentation/planar_segmentation.hpp"

namespace planar_segmentation {
PlanarSegmentationNode::PlanarSegmentationNode(
    const rclcpp::NodeOptions &options)
    : Node("planar_segmentation", options),
      cloud_subscriber_{create_subscription<PointCloudMsg>(
          "input", 10,
          std::bind(&PlanarSegmentationNode::cloudCallback, this,
                    std::placeholders::_1))},
      ground_plane_cloud_publisher_{
          create_publisher<PointCloudMsg>("ground_output", 10)},
      obst_plane_cloud_publisher_{
          create_publisher<PointCloudMsg>("obst_output", 10)} {
  RCLCPP_INFO(this->get_logger(), "planar_segmentation node has been created.");

  max_iters_ = static_cast<int>(declare_parameter("max_iterations").get<int>());
  distance_threshold_ =
      static_cast<float>(declare_parameter("distance_threshold").get<float>());
}

void PlanarSegmentationNode::cloudCallback(const PointCloudMsg::SharedPtr msg) {
  auto init_time = std::chrono::system_clock::now();

  // convert from rosmsg to pcl
  // create pcl pointXYZI object
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*msg, *input_cloud);

  pcl::SACSegmentation<pcl::PointXYZI> seg;

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iters_);
  seg.setDistanceThreshold(static_cast<double>(distance_threshold_));

  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  // create two new clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr obst_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  // create ExtractIndices obj
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);

  // set negative false for ground plane
  extract.setNegative(false);
  extract.filter(*plane_cloud);

  // set negative true for obstacle plane
  extract.setNegative(true);
  extract.filter(*obst_cloud);

  // now convert everything back to ros2 msg types
  PointCloudMsg plane_cloud_msg, obst_cloud_msg;
  pcl::toROSMsg(*plane_cloud, plane_cloud_msg);
  pcl::toROSMsg(*obst_cloud, obst_cloud_msg);

  auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now() - init_time)
                            .count();

  RCLCPP_INFO(this->get_logger(),
              "Planar Segmentation callback finished in %d ms", execution_time);

  // publish
  ground_plane_cloud_publisher_->publish(plane_cloud_msg);
  obst_plane_cloud_publisher_->publish(obst_cloud_msg);
}
}  // namespace planar_segmentation

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(planar_segmentation::PlanarSegmentationNode)