import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    amrj16_0_controller_yaml = os.path.join(get_package_share_directory('amrj16_path_planner_server'), 'config', 'amrj16_0_controller.yaml')
    amrj16_0_bt_navigator_yaml = os.path.join(get_package_share_directory('amrj16_path_planner_server'), 'config', 'amrj16_0_bt_navigator.yaml')
    amrj16_0_planner_yaml = os.path.join(get_package_share_directory('amrj16_path_planner_server'), 'config', 'amrj16_0_planner_server.yaml')
    amrj16_0_recovery_yaml = os.path.join(get_package_share_directory('amrj16_path_planner_server'), 'config', 'amrj16_0_recovery.yaml')

    
    return LaunchDescription([     
        Node(
            namespace = 'amrj16_0',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[amrj16_0_controller_yaml]),

        Node(
            namespace = 'amrj16_0',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[amrj16_0_planner_yaml]),
            
        Node(
            namespace = 'amrj16_0',
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[amrj16_0_recovery_yaml],
            output='screen'),

        Node(
            namespace = 'amrj16_0',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[amrj16_0_bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='amrj16_0_lifecycle_manager_pathplanner_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['amrj16_0_planner_server',
                                        'amrj16_0_controller_server',
                                        'amrj16_0_recoveries_server',
                                        'amrj16_0_bt_navigator']}])
    ])
