"""Production on the Orin: the companion alone, fed by TavSLAM, plus spine_stats.

  ros2 launch spine_bringup orin.launch.py fused_pose_topic:=<TavSLAM FusedPose topic>

The companion subscribes to spine_interfaces/FusedPose. fused_pose_topic must name the topic on
which the TavSLAM side publishes that type (docs/spine-link.md section 3.3).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('spine_bringup'), 'config', 'spine.yaml')
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('fused_pose_topic', default_value='fused_pose',
                              description='TavSLAM output topic (spine_interfaces/FusedPose)'),
        DeclareLaunchArgument('rio_address', default_value='10.105.83.2'),
        DeclareLaunchArgument('nt_server', default_value='10.105.83.2'),
        DeclareLaunchArgument('bind_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        ComposableNodeContainer(
            name='spine_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='spine_link',
                    plugin='spine::SpineCompanionNode',
                    name='spine_companion',
                    parameters=[params, {
                        'rio_address': LaunchConfiguration('rio_address'),
                        'bind_address': LaunchConfiguration('bind_address')}],
                    remappings=[('fused_pose', LaunchConfiguration('fused_pose_topic'))],
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ],
            output='screen'),
        Node(package='spine_stats', executable='spine_stats', name='spine_stats',
             parameters=[params, {'nt_server': LaunchConfiguration('nt_server')}],
             output='screen'),
    ])
