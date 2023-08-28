import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    multibot_robot_dir = get_package_share_directory("multibot_robot")

    # Gazebo
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(multibot_robot_dir, 'launch',
                                                'spawn_robot_launch.py')),
        launch_arguments={
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'robot_name': LaunchConfiguration('robot_name'),
            'sdf_file': LaunchConfiguration('sdf_file'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'Y': LaunchConfiguration('Y')
        }.items()
    )

    # Robot Node
    multibot_robot_cmd = Node(
        package='multibot_robot',
        namespace=LaunchConfiguration('robot_namespace'),
        executable='robot',
        name='robot',
        parameters=[{
            'namespace': LaunchConfiguration('robot_namespace'),
            'linear_tolerance': LaunchConfiguration('linear_tolerance'),
            'angular_tolerance': LaunchConfiguration('angular_tolerance'),
            'Kx': LaunchConfiguration('Kx'),
            'Ky': LaunchConfiguration('Ky'),
            'Ktheta': LaunchConfiguration('Ktheta')
        }],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(spawn_robot_cmd)
    ld.add_action(multibot_robot_cmd)

    return ld