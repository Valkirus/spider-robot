import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    teleop_joy = Node(
        package='spiderproj_teleop_joy',
        executable='spiderproj_teleop_joy',
    )
    ld.add_action(teleop_joy)    
    
    return ld