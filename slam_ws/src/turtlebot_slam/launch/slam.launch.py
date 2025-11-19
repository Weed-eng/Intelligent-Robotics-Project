from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot_slam')
    param_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[param_file, {'use_sim_time': True}]
    )

    return LaunchDescription([slam_node])
