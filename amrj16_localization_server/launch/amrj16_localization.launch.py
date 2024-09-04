from launch import LaunchDescription
from launch_ros.actions import Node
import os
from  ament_index_python.packages import get_package_share_directory

map_filename = os.path.join(get_package_share_directory('amrj16_map_server'), 'config', 'mechanical_workshop.yaml')
amrj16_nav2_yaml = os.path.join(get_package_share_directory('amrj16_localization_server'), 'config', 'amrj16_amcl_config.yaml')
# amrj17_nav2_yaml = os.path.join(get_package_share_directory('amrj17_localization_server'), 'config', 'amrj17_amcl_config.yaml')
# rviz_path = os.path.join(get_package_share_directory('amrj16_localization_server'), 'config', 'amrj16_localization.rviz')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package ='nav2_map_server',
            executable ='map_server',
            name = 'map_server',
            output ='screen',
            parameters = [{'use_sim_time': True},
                          {'topic_name':"map"},
                          {'frame_id':"map"},
                          {'yaml_filename': map_filename}]
            ), 

        Node(
            namespace='amrj16',
            package ='nav2_amcl',
            executable ='amcl',
            name = 'amcl',
            output ='screen',
            parameters = [amrj16_nav2_yaml]
            ), 
       
       # multi-robots navigation
       # Node(
       #     namespace='amrj17',
       #     package ='nav2_amcl',
       #     executable ='amcl',
       #     name = 'amcl',
       #     output ='screen',
       #     parameters = [amrj17_nav2_yaml]
       #     ), 
             
        Node(           

            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_multi_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server','amrj16/amcl']}]  #,'tb3_1/amcl'

        
            )  #,

        #Node(
        #   package='localization_server',
        #    executable='initial_pose_pub_exe',
        #    output='screen'),

        #Node(
        #    package='rviz2',
        #    namespace='',
        #    executable='rviz2',
        #    name='rviz2',    
        #    arguments=['-d' + rviz_path]
        #    )
            
    ])