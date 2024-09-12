import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    map_filename = os.path.join(get_package_share_directory('amrj16_map_server'), 'config', 'utimac_environment_map.yaml')
    nav2_yaml = os.path.join(get_package_share_directory('amrj16_localization_server'), 'config', 'amrj16_amcl_config.yaml')
    #rviz_path = os.path.join(get_package_share_directory('localization_server'), 'config', 'planner.rviz')
    controller_yaml = os.path.join(get_package_share_directory('amrj16_path_planner_server'), 'config', 'amrj16_controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('amrj16_path_planner_server'), 'config', 'amrj16_bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('amrj16_path_planner_server'), 'config', 'amrj16_planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('amrj16_path_planner_server'), 'config', 'amrj16_recovery.yaml')

    
    return LaunchDescription([   

        Node(
            package ='nav2_map_server',
            executable ='map_server',
            name = 'map_server',
            output ='screen',
            parameters = [{'use_sim_time': True},
                          {'yaml_filename': map_filename}]
            ), 

        Node(
            package ='nav2_amcl',
            executable ='amcl',
            name = 'amcl',
            output ='screen',
            parameters = [nav2_yaml]
            ), 
               
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}])

   
   
    ])
