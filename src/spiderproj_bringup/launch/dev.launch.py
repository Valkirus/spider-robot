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

    rviz_config_path = os.path.join(get_package_share_path('spiderproj_description'),
                                    'rviz', 'config.rviz')

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

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{'source_list': ['/kinematics/joint_states']}]
    )

    # Teleop Launch File
    teleop_joy = Node(
        package='spiderproj_teleop_joy',
        executable='spiderproj_teleop_joy',
    )

    # Kinematics Node
    kinematics_node = Node(
        package='spiderproj_kinematics',
        executable='spiderproj_kinematics',
        name='spiderproj_kinematics',
    )

    #Rviz2 Node
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    # Final Launch Description
    return LaunchDescription([
        global_param_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz2_node,
        #kinematics_node,
        teleop_joy
    ])
