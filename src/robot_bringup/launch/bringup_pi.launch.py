from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # serial esp32
    serial_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('motor_control'),
                'launch',
                'serial_bridge.launch.py'
            )
        )
    )

    # kinematic 
    kinematic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('motor_control'),
                'launch',
                'kinematic.launch.py'
            )
        )
    )

    #Yd lidar x3 pro
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            )
        )
    )

    # mapping
    # cartographer_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('robot_mapping'),
    #             'launch'
    #             'cartographer.launch.py'
    #         )
    #     )
    # )
    

    return LaunchDescription([
        serial_bridge_launch,
        kinematic_launch,
        ydlidar_launch,
        # cartographer_launch,
    ])

    # save map
    #ros2 run nav2_map_server map_saver_cli -f map
