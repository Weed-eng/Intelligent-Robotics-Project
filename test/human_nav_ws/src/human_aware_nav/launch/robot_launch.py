import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_name = 'human_aware_nav'
    
    # 1. 查找世界文件
    world_file = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'worlds',
        'human_env.wbt'
    ])

    # 2. 配置 Webots 启动器
    webots = WebotsLauncher(
        world=world_file,
        ros2_supervisor=True
    )

    # 3. 配置机器人驱动
    robot_description_path = os.path.join(
        get_package_share_directory(package_name),
        'resource',
        'my_robot.urdf'
    )

    # 启动机器人控制器节点
    my_robot_driver = WebotsController(
        robot_name='MyRobot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
