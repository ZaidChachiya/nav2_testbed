import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('testbed_navigation')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # Shared parameters for all navigation nodes (YAML + sim time override)
    nav_params = [params_file, {'use_sim_time': True}]

    # Nodes that must be managed by the lifecycle manager
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    return LaunchDescription([
        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=nav_params,
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

        # Path Smoother
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=nav_params
        ),

        # Planner Server (global path)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=nav_params
        ),

        # Behavior Server (recoveries, spin, etc.)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=nav_params
        ),

        # BT Navigator (the /navigate_to_pose action)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=nav_params
        ),

        # Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=nav_params
        ),

        # Velocity Smoother (final cmd_vel to robot)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=nav_params,
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel')
            ]
        ),

        # Lifecycle Manager (activates everything automatically)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': lifecycle_nodes}
            ]
        ),
    ])