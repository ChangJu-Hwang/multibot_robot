from launch import LaunchDescription, LaunchContext
from launch.substitutions import Command, LaunchConfiguration

import launch.actions
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-robot_namespace', LaunchConfiguration('robot_namespace'),
                '-entity', LaunchConfiguration('robot_name'),
                '-file', LaunchConfiguration('sdf_file'),
                '-reference_frame', "map",
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-Y', LaunchConfiguration('Y')
            ]
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[{'frame_prefix': LaunchConfiguration('frame_prefix')},
                        {'use_sim_time': True}],
            arguments=[LaunchConfiguration('urdf_file')]
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[{'frame_prefix': LaunchConfiguration('frame_prefix')},
                        {'use_sim_time': True}],
            arguments=[LaunchConfiguration('urdf_file')]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[LaunchConfiguration('x'),
                       LaunchConfiguration('y'),
                       '0',
                       LaunchConfiguration('Y'),
                       '0','0',
                       'map', 'Robot1/odom']
        )
    ])
    