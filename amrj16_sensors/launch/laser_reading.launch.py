from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amrj16_sensors',
            executable='laser_reader_node',
            output='screen'),
    ])
