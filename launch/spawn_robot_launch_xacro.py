import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

def generate_launch_description():
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='joint_state_publisher',
        namespace='Robot1',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model_file'), ' robot_name:=', LaunchConfiguration('robot_name')])},
                    {'frame_prefix': LaunchConfiguration('frame_prefix')},
                    {'use_sim_time': True}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='Robot1',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model_file'), ' robot_name:=', LaunchConfiguration('robot_name')])},
                    {'frame_prefix': LaunchConfiguration('frame_prefix')},
                    {'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='Robot1',
        output='screen',
        arguments=[
            '-robot_namespace', LaunchConfiguration('robot_namespace'),
            '-entity', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-reference_frame', "map",
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-Y', LaunchConfiguration('Y')
        ]
    )

    static_transform_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[LaunchConfiguration('x'),
                       LaunchConfiguration('y'),
                       '0',
                       LaunchConfiguration('Y'),
                       '0','0',
                       'map', LaunchConfiguration('odom_frame')]
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity)
    ld.add_action(static_transform_publisher)

    return ld