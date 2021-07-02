#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "rclcpp/rclcpp.hpp"

namespace pointcloud_fusion {
class PointCloudFusionNode : public rclcpp::Node {
 public:
  PointCloudFusionNode(const rclcpp::NodeOptions &options);

 private:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<PointCloudMsg,
                                                      PointCloudMsg>;

  // main callback function
  void pointCloudCallback(const PointCloudMsg::ConstSharedPtr &msg1,
                          const PointCloudMsg::ConstSharedPtr &msg2);

  void concatenatePointCloud(const PointCloudMsg &pc_in, PointCloudMsg &pc_out,
                             uint32_t &concat_idx,
                             const Eigen::Affine3f &affine_tf);

  void convertToAffine3f(const geometry_msgs::msg::Transform &tf,
                         Eigen::Affine3f &af);

  Eigen::Affine3f affine3f_front_lidar_;
  Eigen::Affine3f affine3f_rear_lidar_;

  // message filter synchronizer object
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>>
      point_cloud_synchronizer_;

  // final fused point cloud publisher object
  rclcpp::Publisher<PointCloudMsg>::SharedPtr fused_point_cloud_publisher_;

  // need two subscribers (front and rear lidars)
  std::unique_ptr<message_filters::Subscriber<PointCloudMsg>>
      front_lidar_subscriber_;
  std::unique_ptr<message_filters::Subscriber<PointCloudMsg>>
      rear_lidar_subscriber_;

  // final fused point cloud msg object
  PointCloudMsg fused_point_cloud_;

  std::string fused_frame_name_;
  uint32_t fused_point_cloud_max_capacity_;
};
}  // namespace pointcloud_fusion