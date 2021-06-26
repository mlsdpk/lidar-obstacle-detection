#include "rclcpp/rclcpp.hpp"

#include "pointcloud_fusion/pointcloud_fusion.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PointCloudFusionNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}