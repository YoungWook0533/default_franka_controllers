import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Add the path to the `utils` folder
package_share = get_package_share_directory('dyros_fr3_controllers')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'dyros_fr3_controllers', 'utils')
sys.path.append(os.path.abspath(utils_path))

from launch_utils import load_yaml


def generate_robot_nodes(context):
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    controller_name = LaunchConfiguration('controller_name').perform(context)
    configs = load_yaml(config_file)
    nodes = []
    for item_name, config in configs.items():
        namespace = config['namespace']
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('dyros_fr3_controllers'), 'launch', 'franka.launch.py'
                    ])
                ),
                launch_arguments={
                    'arm_id': str(config['arm_id']),
                    'arm_prefix': str(config['arm_prefix']),
                    'namespace': str(namespace),
                    'urdf_file': str(config['urdf_file']),
                    'robot_ip': str(config['robot_ip']),
                    'load_gripper': str(config['load_gripper']),
                    'use_fake_hardware': str(config['use_fake_hardware']),
                    'fake_sensor_commands': str(config['fake_sensor_commands']),
                    'joint_state_rate': str(config['joint_state_rate']),
                }.items(),
            )
        )
        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=[controller_name, '--controller-manager-timeout', '30'],
                parameters=[PathJoinSubstitution([
                    FindPackageShare('dyros_fr3_controllers'), 'config', "controllers.yaml",

                ])],
                output='screen',
            )
        )
    if any(str(config.get('use_rviz', 'false')).lower() == 'true' for config in configs.values()):
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['--display-config', PathJoinSubstitution([
                    FindPackageShare('franka_description'), 'rviz', 'visualize_franka.rviz'
                ])],
                output='screen',
            )
        )
    return nodes

# The generate_launch_description function is the entry point (like "main")
# It is called by the ROS 2 launch system when the launch file is executed.
# via: ros2 launch franka_bringup example.launch.py ARGS...
# This function must return a LaunchDescription object containing nodes to be launched.
# it calls the generate_robot_nodes function to get the list of nodes to be launched.


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('dyros_fr3_controllers'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        DeclareLaunchArgument(
            'controller_name',
            description='Name of the controller to spawn (required, no default)',
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
