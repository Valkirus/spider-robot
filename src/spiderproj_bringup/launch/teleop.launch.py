import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    joy_node = Node(
        package='spiderproj_nrf24',
        executable='rf24_node',
    ) 
    ld.add_action(joy_node)
    
    teleop_joy = Node(
        package='spiderproj_teleop_joy',
        executable='spiderproj_teleop_joy',
    )
    ld.add_action(teleop_joy)    
    
    return ld