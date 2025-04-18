from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('spiderproj_description'),
                             'urdf', 'spiderproj.xacro')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    global_parameters = os.path.join(
        get_package_share_directory('spiderproj_bringup'),
        'config',
        'global_params.yaml'
    )

    # Global parameter node
    global_param_node = Node(
        package='spiderproj_bringup',
        executable='global_parameter_server',
        name='global_parameter_server',
        parameters=[global_parameters, {'robot_description': robot_description}]
    )
    
    # Teleop Launch File
    teleop_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('spiderproj_bringup'),
                "launch/teleop.launch.py"
            )
        )
    )

    # Kinematics Node
    kinematics_node = Node(
        package='spiderproj_kinematics',
        executable='spiderproj_kinematics',
        name='spiderproj_kinematics',
    )

    # Final Launch Description
    return LaunchDescription([
        global_param_node,
        kinematics_node,
        teleop_launch_file,
    ])
