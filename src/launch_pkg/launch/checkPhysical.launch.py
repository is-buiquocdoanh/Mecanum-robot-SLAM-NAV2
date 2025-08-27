import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # -- get param in share package
    # config = os.path.join(
    #   get_package_share_directory('launch_pkg'),
    #   'config',
    #   'param.yaml'
    # )

    # -- get param in src package
    # config = os.path.join(os.getcwd(), "src", "launch_pkg", "config", "param.yaml")
    config = "/home/stivietnam/ros2_ws/src/launch_pkg/config/param.yaml"
    
    return LaunchDescription([
        Node(
            package='launch_pkg',
            executable='check_physical',
            name='check_physical',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        )
    ])
