"""Bench / fake coprocessor: companion + synthetic pose node (one process, intra-process comms)
plus spine_stats.

  ros2 launch spine_bringup bench.launch.py                     # real RIO at 10.105.83.2
  ros2 launch spine_bringup bench.launch.py sim:=true           # RIO sim on this machine
  ros2 launch spine_bringup bench.launch.py rio_address:=10.105.83.2 nt_server:=10.105.83.2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _setup(context):
    def arg(name):
        return context.launch_configurations[name]

    sim = arg('sim').lower() in ('true', '1', 'yes')
    rio = arg('rio_address') or ('127.0.0.1' if sim else '10.105.83.2')
    nt = arg('nt_server') or rio
    params = arg('params_file')
    ipc = [{'use_intra_process_comms': True}]

    container = ComposableNodeContainer(
        name='spine_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='spine_link',
                plugin='spine::SpineCompanionNode',
                name='spine_companion',
                parameters=[params, {'rio_address': rio, 'bind_address': arg('bind_address')}],
                extra_arguments=ipc),
            ComposableNode(
                package='spine_link',
                plugin='spine::SyntheticPoseNode',
                name='spine_synthetic_pose',
                parameters=[params],
                extra_arguments=ipc),
        ],
        output='screen')

    actions = [container]
    if arg('stats').lower() in ('true', '1', 'yes'):
        actions.append(Node(
            package='spine_stats', executable='spine_stats', name='spine_stats',
            parameters=[params, {'nt_server': nt}], output='screen'))
    return actions


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('spine_bringup'), 'config', 'spine.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='RIO runs in simulation on this machine (127.0.0.1)'),
        DeclareLaunchArgument('rio_address', default_value='',
                              description='Stream 2 destination; default 10.105.83.2, or '
                                          '127.0.0.1 with sim:=true'),
        DeclareLaunchArgument('nt_server', default_value='',
                              description='NT4 server for spine_stats; default rio_address'),
        DeclareLaunchArgument('bind_address', default_value='0.0.0.0',
                              description='Interface Streams 1 and 3 are received on'),
        DeclareLaunchArgument('stats', default_value='true',
                              description='Also run spine_stats'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        OpaqueFunction(function=_setup),
    ])
