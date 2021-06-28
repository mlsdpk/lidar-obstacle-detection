import launch
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os

point_cloud_fusion_node_pkg_prefix = get_package_share_directory('pointcloud_fusion')
point_cloud_fusion_node_param_file = os.path.join(point_cloud_fusion_node_pkg_prefix,
                                                  'config/params.yaml')


def generate_launch_description():
    pointcloud_fusion_node = Node(
        package='pointcloud_fusion',
        executable='pointcloud_fusion_node',
        parameters=[point_cloud_fusion_node_param_file],
        remappings=[
            ("output", "/lidar_fused/transformed_points_raw"),
            ("input1", "/lidar_front/points_raw"),
            ("input2", "/lidar_rear/points_raw")
        ]
    )

    return launch.LaunchDescription([pointcloud_fusion_node])
