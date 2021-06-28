#include "pointcloud_fusion/pointcloud_fusion.hpp"

PointCloudFusionNode::PointCloudFusionNode()
    : Node("pointcloud_fusion"),
      fused_point_cloud_publisher_{create_publisher<PointCloudMsg>("output", 10)},
      fused_frame_name_{declare_parameter("fused_frame_name").get<std::string>()},
      fused_point_cloud_max_capacity_{static_cast<uint32_t>(declare_parameter("fused_point_cloud_max_capacity").get<int>())} {
    RCLCPP_INFO(this->get_logger(), "pointcloud_fusion node has been created.");

    // initialize fused point cloud message
    fused_point_cloud_.height = 1U;
    fused_point_cloud_.is_bigendian = false;
    fused_point_cloud_.is_dense = false;
    // TODO: pass frame name and cloud capacity as ros 2 parameter
    fused_point_cloud_.header.frame_id = fused_frame_name_;
    sensor_msgs::PointCloud2Modifier modifier(fused_point_cloud_);
    modifier.setPointCloud2Fields(
        4,
        "x", 1U, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1U, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1U, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1U, sensor_msgs::msg::PointField::FLOAT32
    );
    modifier.resize(fused_point_cloud_max_capacity_);

    // initialize front and rear lidar subscriber objects
    // TODO: pass topic names as ros 2 parameters
    front_lidar_subscriber_ = std::make_unique<message_filters::Subscriber<PointCloudMsg>>(
        this,
        "input1"
    );
    rear_lidar_subscriber_ = std::make_unique<message_filters::Subscriber<PointCloudMsg>>(
        this,
        "input2"
    );

    // initialize message filter stuffs
    point_cloud_synchronizer_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10),
        *front_lidar_subscriber_,
        *rear_lidar_subscriber_
    );

    point_cloud_synchronizer_->registerCallback(
        std::bind(
            &PointCloudFusionNode::pointCloudCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );
    
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
    if (std::chrono::nanoseconds(latest_timestamp.nanosec) < std::chrono::nanoseconds(msg2->header.stamp.nanosec)) {
        latest_timestamp = msg2->header.stamp;
    }

    // get total size of point clouds
    //auto total_size = msg1->width + msg2->width;

    // TODO: check to make sure total size is not greater than 
    //       fused_point_cloud_ msg size

    uint32_t point_cloud_idx = 0;
    concatenatePointCloud(*msg1, fused_point_cloud_, point_cloud_idx);
    concatenatePointCloud(*msg2, fused_point_cloud_, point_cloud_idx);

    // resize and publish
    modifier.resize(point_cloud_idx);
    
    //RCLCPP_INFO(this->get_logger(), "here %d.", point_cloud_idx);
    fused_point_cloud_.header.stamp = latest_timestamp;
    fused_point_cloud_publisher_->publish(fused_point_cloud_);
}

void PointCloudFusionNode::concatenatePointCloud(
    const PointCloudMsg & pc_in,
    PointCloudMsg & pc_out,
    uint32_t & concat_idx) {

    // TODO: add error handlings

    // TODO: float needs to be 32-bit
    // assert if it is not true
    // get const iterator from input msg
    sensor_msgs::PointCloud2ConstIterator<float> x_it_in(pc_in, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_it_in(pc_in, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_it_in(pc_in, "z");
    sensor_msgs::PointCloud2ConstIterator<float> intensity_it_in(pc_in, "intensity"); 

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

    while (x_it_in != x_it_in.end() &&
        y_it_in != y_it_in.end() &&
        z_it_in != z_it_in.end() &&
        intensity_it_in != intensity_it_in.end()) {

        // input point
        auto x_in = *x_it_in;
        auto y_in = *y_it_in;
        auto z_in = *z_it_in;
        auto intensity_in = *intensity_it_in;

        // add input to output msg if its idx is not at the end of iter
        if (x_it_out != x_it_out.end() &&
            y_it_out != y_it_out.end() &&
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
