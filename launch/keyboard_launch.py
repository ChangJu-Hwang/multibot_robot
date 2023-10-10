import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

import yaml

def generate_launch_description():

    multibot_robot_dir = get_package_share_directory("multibot_robot")

    with open(os.path.join(multibot_robot_dir, 'robot', 'robotConfig.yaml')) as robotConfig_params:
        robotConfig_params = yaml.load(robotConfig_params, Loader=yaml.Loader)
        robotConfig_params = robotConfig_params['/**']['ros__parameters']['ISR_M2']

    # ISR_M2 Driver
    isr_m2_node_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('multibot_driver'), 'launch',
                                                   'isr_m2_node_launch.py')),
        launch_arguments={
            'robot_name': 'ISR_M2',
            'odom_frame': 'ISR_M2' + '/' + robotConfig_params['odometry']['frame_id'],
            'base_frame': 'ISR_M2' + '/' + robotConfig_params['odometry']['child_frame_id'],
            'laser_frame': 'ISR_M2' + '/' + robotConfig_params['laser']['frame_id'],
            'laser_offset_x': str(robotConfig_params['laser']['offset_x']),
            'laser_offset_y': str(robotConfig_params['laser']['offset_y']),
            'laser_offset_z': str(robotConfig_params['laser']['offset_z'])
        }.items()
    )

    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace='ISR_M2',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['amcl']}
        ]
    )

    # Robot Node
    multibot_robot_cmd = Node(
        package='multibot_robot',
        namespace='ISR_M2',
        executable='robot',
        name='robot',
        parameters=[
            os.path.join(multibot_robot_dir, 'robot', 'robotConfig.yaml'),
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(isr_m2_node_cmd)
    # ld.add_action(lifecycle_manager_cmd)
    ld.add_action(multibot_robot_cmd)

    return ld