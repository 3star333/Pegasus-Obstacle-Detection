#!/usr/bin/env python3
"""
Full Pegasus Obstacle Avoidance Stack Launch File

Starts all three nodes together:
  1. obstacle_detector  — LiDAR + Kalman tracking + zone classification
  2. costmap_node       — OccupancyGrid from tracked obstacles
  3. rrt_star_planner   — RRT* path planner driven by avoidance commands

Usage:
    ros2 launch costmap full_stack.launch.py
    ros2 launch costmap full_stack.launch.py use_sim_time:=false

Topic graph:
    /lidar/points          → obstacle_detector → /avoidance_command   → rrt_star_planner
                                               → /obstacle_velocity   → rrt_star_planner
                                               → /obstacle_detected
                                               → /obstacle_distance
                                               → /costmap/update_trigger → costmap_node
    /lidar/points          → costmap_node      → /costmap/grid        → rrt_star_planner
    /vehicle/pose          → rrt_star_planner
    /vehicle/pose          → costmap_node
    /planning/goal         → rrt_star_planner  → /planning/path
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true for Gazebo)',
    )

    sim_time = LaunchConfiguration('use_sim_time')

    # ── Obstacle Detector ─────────────────────────────────────────────
    obstacle_detector_node = Node(
        package='obstacle_detection',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('obstacle_detection'), 'config', 'params.yaml'
            ]),
            {'use_sim_time': sim_time},
        ],
        emulate_tty=True,
    )

    # ── Costmap Node ──────────────────────────────────────────────────
    costmap_node = Node(
        package='costmap',
        executable='costmap_node',
        name='costmap_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('costmap'), 'config', 'costmap_params.yaml'
            ]),
            {'use_sim_time': sim_time},
        ],
        emulate_tty=True,
    )

    # ── RRT* Planner ──────────────────────────────────────────────────
    rrt_star_node = Node(
        package='rrt_star_planner',
        executable='rrt_star_planner',
        name='rrt_star_planner',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rrt_star_planner'), 'config', 'rrt_star_params.yaml'
            ]),
            {'use_sim_time': sim_time},
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        use_sim_time_arg,
        obstacle_detector_node,
        costmap_node,
        rrt_star_node,
    ])
