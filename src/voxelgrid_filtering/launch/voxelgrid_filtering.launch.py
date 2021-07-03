import launch
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os

voxelgrid_filtering_node_pkg_prefix = get_package_share_directory('voxelgrid_filtering')
voxelgrid_filtering_node_param_file = os.path.join(voxelgrid_filtering_node_pkg_prefix,
                                                  'config/params.yaml')

def generate_launch_description():
    voxelgrid_filtering_node = Node(
        package='voxelgrid_filtering',
        executable='voxelgrid_filtering_node_exe',
        parameters=[voxelgrid_filtering_node_param_file],
        remappings=[
            ("output", "/lidar_fused/transformed_points_downsampled"),
            ("input", "/lidar_fused/transformed_points_raw")
        ]
    )

    return launch.LaunchDescription([voxelgrid_filtering_node])
