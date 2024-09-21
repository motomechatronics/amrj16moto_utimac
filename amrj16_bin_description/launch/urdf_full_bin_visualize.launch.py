import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import xacro

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    #urdf_file = 'amrj16.urdf'
    #xacro_file = "amrj16_main.xacro"
    xacro_file = "amrj16_full_bin.xacro"
    package_description = "amrj16_bin_description"    
    # Boolean flag to enable or disable joint_state_publisher_gui
    use_joint_state_publisher_gui = False  # Set this to False if you don't want to launch it

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")    
    #robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", xacro_file)

    
    rsp = Node(   
        namespace="full_bin",    
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher', 
        emulate_tty=True,       
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
       
    )

    nodes_to_launch = [rsp]

       
    # Robot State Publisher
    # Joint State Publisher_gui

    if use_joint_state_publisher_gui:
        joint_state_publisher_gui_node = Node(
            namespace="full_bin", 
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',            
            name='joint_state_publisher_node',
            emulate_tty=True,
            output="screen"
        )
        nodes_to_launch.append(joint_state_publisher_gui_node)

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'full_bin_rviz2_config.rviz')


    rviz2_node = Node(  
                     
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',            
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir]
    )

    nodes_to_launch.append(rviz2_node)

    # create and return launch description object
    return LaunchDescription(nodes_to_launch)