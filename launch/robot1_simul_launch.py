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
    target = 'robot1.yaml'

    multibot_robot_dir = get_package_share_directory("multibot_robot")

    with open(os.path.join(multibot_robot_dir, 'robot', target)) as robot_params:
        robot_params = yaml.load(robot_params, Loader=yaml.Loader)
        robot_params = robot_params['/**']['ros__parameters']
        with open(os.path.join(multibot_robot_dir, 'robot', 'robotConfig.yaml')) as robotConfig_params:
            robotConfig_params = yaml.load(robotConfig_params, Loader=yaml.Loader)
            robotConfig_params = robotConfig_params['/**']['ros__parameters'][robot_params['type']]

    # Fake Node
    isr_m2_fake_node_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('multibot_driver'), 'launch',
                                                   'isr_m2_fake_node_launch.py')),
        launch_arguments={
            'robot_name': robot_params['name'],
            'frame_prefix': robot_params['name'] + '/',
            'odom_frame': robot_params['name'] + '/odom',
            'base_frame': robot_params['name'] + '/base_link',
            'x': str(robot_params['spawn']['x']),
            'y': str(robot_params['spawn']['y']),
            'Y': str(robot_params['spawn']['theta'])
        }.items()
    )
    
    # AMCL
    rviz_config_dir = os.path.join(
        multibot_robot_dir,
        'rviz',
        'multibot_robot.rviz'
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_params['name'],
        arguments=['-d', rviz_config_dir],
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen',
        remappings=[('/initialpose', '/'+robot_params['name']+'/initialpose')]
    )

    amcl_params = RewrittenYaml(
        source_file=os.path.join(multibot_robot_dir, 'params', 'amcl.yaml'),
        root_key=robot_params['name'],
        param_rewrites={
            'base_frame_id': robot_params['name']+'/base_link',
            'odom_frame_id': robot_params['name']+'/odom',
            'scan_topic': '/'+robot_params['name']+'/scan'
        },
        convert_types=True
    )

    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=robot_params['name'],
        output='screen',
        parameters=[
            amcl_params,
            {'use_sim_time': True}
            ],
        remappings=[
            ('/initialpose', '/'+robot_params['name']+'/initialpose')
            ]
    )

    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=robot_params['name'],
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
        namespace=robot_params['name'],
        executable='robot',
        name='robot',
        parameters=[
            os.path.join(multibot_robot_dir, 'robot', 'robotConfig.yaml'),
            os.path.join(multibot_robot_dir, 'robot', target),
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(isr_m2_fake_node_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(multibot_robot_cmd)

    return ld