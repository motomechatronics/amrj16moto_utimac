#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random

from launch_ros.actions import Node
from launch import LaunchDescription


# this is the function launch  system will look for


def generate_launch_description():


    # Position and orientation
    # [X, Y, Z]
    position = [24.3, 1.75, 0.015]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, -3.14]
    # Base Name or robot
    entity_name = "full_bin"   

    # Spawn ROBOT Set Gazebo
    spawn_bin = Node(       
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
                   '-topic', entity_name + '/robot_description'
                   ]
    )

    

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_bin,
        ]
    )
