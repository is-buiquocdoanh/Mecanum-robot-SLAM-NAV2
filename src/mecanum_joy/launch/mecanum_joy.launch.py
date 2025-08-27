from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node đọc joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',   # Đường dẫn thiết bị joystick
                'deadzone': 0.1,           # Deadzone cho cần điều khiển
                'autorepeat_rate': 20.0    # Lặp lại khi giữ nút
            }]
        ),

        # Node điều khiển mecanum
        Node(
            package='mecanum_joy',
            executable='joy_mecanum',
            name='joy_mecanum',
            output='screen'
        )
    ])
