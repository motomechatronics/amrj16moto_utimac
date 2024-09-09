#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random

from launch_ros.actions import Node
from launch import LaunchDescription


# this is the function launch  system will look for


def generate_launch_description():


    # Position and orientation
    # [X, Y, Z]
    position = [-5.3, 6.2, 0.2]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, -1.5707]
    # Base Name or robot
    robot_name_1 = "amrj16_0"
    # robot_name_2 = "amrj16_1"

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                    robot_name_1,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', robot_name_1+'/robot_description'
                   ]
    )

    

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_robot,
        ]
    )
