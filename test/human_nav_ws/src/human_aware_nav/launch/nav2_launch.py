# nav2 launch file for human-aware navigation
# uses saved map + AMCL for localization, DWB for local planning
# DWB outputs to /cmd_vel_raw, adaptive_safety adds predictive layer to /cmd_vel

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_name = 'human_aware_nav'
    package_dir = get_package_share_directory(package_name)

    # paths to config and map files
    nav2_params_file = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(package_dir, 'maps', 'test_map.yaml')

    # launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')

    # rewrite params to set map file path
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time
        },
        convert_types=True
    )

    # map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('map', 'map')]
    )

    # AMCL localization node
    # uses raw scan - particle filter handles dynamic obstacles
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params]
    )

    # planner server (global path planning)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params]
    )

    # controller server (DWB local planner)
    # outputs to /cmd_vel_raw, adaptive_safety filters to /cmd_vel
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_raw')]
    )

    # behavior server (recovery behaviors)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel_raw')]
    )

    # bt navigator (behavior tree orchestration)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params]
    )

    # lifecycle manager to manage all nav2 nodes
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start nav2 stack'
        ),
        map_server_node,
        amcl_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node
    ])
