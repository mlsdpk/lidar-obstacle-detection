import launch
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os

planar_segmentation_node_pkg_prefix = get_package_share_directory('planar_segmentation')
planar_segmentation_node_param_file = os.path.join(planar_segmentation_node_pkg_prefix,
                                                  'config/params.yaml')

def generate_launch_description():
    planar_segmentation_node = Node(
        package='planar_segmentation',
        executable='planar_segmentation_node_exe',
        parameters=[planar_segmentation_node_param_file],
        remappings=[
            ("ground_output", "/ground_points"),
            ("obst_output", "/obstacle_points"),
            ("input", "/lidar_fused/transformed_points_downsampled")
        ]
    )

    return launch.LaunchDescription([planar_segmentation_node])
