from launch import LaunchDescription
from launch_ros.actions import Node
import os
from  ament_index_python.packages import get_package_share_directory

cartographer_config_dir = os.path.join(get_package_share_directory('amrj16_cartographer_slam'), 'config')
configuration_basename = 'cartographer.lua'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package ='cartographer_ros',
            executable ='cartographer_node',
            name = 'cartographer_node',
            output ='screen',
            parameters = [{'use_sim_time': True}],
            arguments = ['-configuration_directory',cartographer_config_dir,
            '-configuration_basename', configuration_basename],
            remappings=[
                ('/cmd_vel', '/amrj16_0/cmd_vel'),
                ('/odom', '/amrj16_0/odom'),
                ('/scan', '/amrj16_0/laserscan'),],

        ),
        
        Node(
            package = 'cartographer_ros',
            executable = 'occupancy_grid_node',
            name = 'occupancy_grid_node',
            output = 'screen',
            parameters = [{'use_sim_time': True}],
            arguments = ['-resolution','0.05','-publish_period_sec','1.0'],
            
        
        ),
            
    ])