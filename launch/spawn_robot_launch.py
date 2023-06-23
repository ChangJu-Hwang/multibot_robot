from launch import LaunchDescription

import launch.actions
import launch_ros.actions

def generate_launch_description():

    remappings = [('odom', 'Agent1')]

    return LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                '-entity', launch.substitutions.LaunchConfiguration('robot_name'),
                '-file', launch.substitutions.LaunchConfiguration('sdf_file'),
                '-reference_frame', "map",
                '-x', launch.substitutions.LaunchConfiguration('x'),
                '-y', launch.substitutions.LaunchConfiguration('y'),
                '-Y', launch.substitutions.LaunchConfiguration('Y')
            ],
        )
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=[launch.substitutions.LaunchConfiguration('x'),
        #                launch.substitutions.LaunchConfiguration('y'),
        #                '0',
        #                launch.substitutions.LaunchConfiguration('Y'),
        #                '0','0',
        #                'map','odom']
        # )
    ])
    