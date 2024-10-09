from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amrj16_localization_server',
            executable='amrj16_record_poi_server_exe',
            output='screen'),
    ])