import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import yaml

def generate_launch_description():
    multibot_robot_dir = get_package_share_directory("multibot_robot")
    cartographer_config_dir = os.path.join(multibot_robot_dir, 'config')
    configuration_basename = 'ISR_M2_lds_2d.lua'

    use_sim_time = True

    robot_name = 'ISR_M2'
    robot_type = 'ISR_M2'

    x = 17.8
    y = 1.1
    Y = 0.0

    with open(os.path.join(multibot_robot_dir, 'robot', 'robotConfig.yaml')) as robotConfig_params:
        robotConfig_params = yaml.load(robotConfig_params, Loader=yaml.Loader)
        robotConfig_params = robotConfig_params['/**']['ros__parameters'][robot_type]

    # Fake Node
    isr_m2_fake_node_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('multibot_driver'), 'launch',
                                                   'isr_m2_fake_node_launch.py')),
        launch_arguments={
            'robot_name': robot_name,
            'robot_type': robot_type,
            'robot_config': os.path.join(multibot_robot_dir, 'robot', 'robotConfig.yaml'),
            'frame_prefix': robot_name + '/',
            'odom_frame': robot_name + '/' + robotConfig_params['odometry']['frame_id'],
            'base_frame': robot_name + '/' + robotConfig_params['odometry']['child_frame_id'],
            'x': str(x),
            'y': str(y),
            'Y': str(Y)
        }.items()
    )
    
    # Robot Node
    multibot_robot_cmd = Node(
        package='multibot_robot',
        namespace=robot_name,
        executable='robot',
        name='robot',
        parameters=[
            os.path.join(multibot_robot_dir, 'robot', 'robotConfig.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Static Transform Publisher
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_name,
        output='screen',
        arguments=[str(x),str(y),'0',str(Y),'0','0',
                   'map', robot_name + '/odom']
    )

    # Mapping
    mapping = Node(
        package='cartographer_ros',
        namespace=robot_name,
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        namespace=robot_name,
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=[
            '-resolution', '0.1',
            '-publish_period_sec', '0.5'
        ]
    )

    #Rviz
    rviz_config_dir = os.path.join(
        multibot_robot_dir,
        'rviz',
        'mapping.rviz'
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_name,
        arguments=['-d', rviz_config_dir],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(isr_m2_fake_node_cmd)
    ld.add_action(multibot_robot_cmd)
    ld.add_action(map_to_odom)

    ld.add_action(mapping)
    ld.add_action(occupancy_grid_node)
    ld.add_action(start_rviz_cmd)

    return ld