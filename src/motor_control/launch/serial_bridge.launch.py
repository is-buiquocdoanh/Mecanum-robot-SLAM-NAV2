import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # -- get param in src package
    # config = os.path.join(os.getcwd(), "src", "launch_pkg", "config", "param.yaml")
    config = "/home/raspi/ros2_ws/src/launch_pkg/config/param.yaml"
    
    return LaunchDescription([
        Node(
            package='motor_control',
            executable='ros_serial_bridge',
            name='ros_serial_bridge',
            output='screen',
            emulate_tty=True
            # parameters=[config]
        )
    ])