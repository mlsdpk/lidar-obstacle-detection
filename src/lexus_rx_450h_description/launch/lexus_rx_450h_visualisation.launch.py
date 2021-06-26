import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

package_path = get_package_share_directory('lexus_rx_450h_description')
# urdf_path = package_path / Path('urdf/lexus_rx_450h.urdf')
# rviz_cfg_path = package_path / Path('config/lexus_rx_450h.rviz')

urdf_file_name = 'lexus_rx_450h.urdf'
rviz_file_name = 'lexus_rx_450h.rviz'
urdf_path = os.path.join(package_path, urdf_file_name)
rviz_cfg_path = os.path.join(package_path, rviz_file_name)


def generate_launch_description():
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_file}]),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_cfg_path)])
    ])
