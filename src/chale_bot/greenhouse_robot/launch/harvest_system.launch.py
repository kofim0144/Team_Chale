"""
harvest_system.launch.py — full harvesting pipeline

Single launch that brings up the complete harvesting system:
  1. Gazebo + ros2_control (chaleBOT_description/launch/gazebo.launch.py)
  2. MoveIt move_group + RViz (chaleBOT_moveIt_config/launch/move_group.launch.py)
  3. Perception: fruit_detector
  4. Arm controller: arm_controller (talks to MoveIt + deletes fruits)
  5. FSM: robot_coordinator (drives + scans + dispatches)

Usage:
    ros2 launch greenhouse_robot harvest_system.launch.py

Recommended for the FIRST run:
    Launch Gazebo and MoveIt SEPARATELY (in their own terminals) so you can
    see their logs cleanly. Then launch ONLY the perception + arm + fsm
    nodes via the manual run commands in INTEGRATION_GUIDE.txt.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg   = get_package_share_directory('chaleBOT_description')
    moveit_pkg = get_package_share_directory('chaleBOT_moveIt_config')
    grobot_pkg = get_package_share_directory('greenhouse_robot')

    registry_path = os.path.join(grobot_pkg, 'config', 'fruit_registry.yaml')

    # Optional: skip Gazebo (it may already be running)
    skip_gazebo_arg = DeclareLaunchArgument(
        'skip_gazebo', default_value='false',
        description='If true, do not launch gazebo.launch.py')

    skip_moveit_arg = DeclareLaunchArgument(
        'skip_moveit', default_value='false',
        description='If true, do not launch move_group.launch.py')

    # 1. Gazebo + ros2_control
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'gazebo.launch.py')),
    )

    # 2. MoveIt move_group + RViz (combined launch)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'move_group.launch.py')),
    )

    # 3. Perception (delayed -- waits for Gazebo's /camera topics)
    perception_nodes = TimerAction(period=8.0, actions=[
        Node(package='greenhouse_robot', executable='fruit_detector',
             name='fruit_detector', output='screen',
             parameters=[{
                 'rgb_topic':         '/camera/color/image_raw',
                 'depth_topic':       '/camera/depth/image_raw',
                 'camera_info_topic': '/camera/color/camera_info',
                 'target_frame':      'base_link',
             }]),
    ])

    # 4. Arm controller (delayed -- needs both MoveIt and Gazebo's delete service)
    arm_node = TimerAction(period=12.0, actions=[
        Node(package='greenhouse_robot', executable='arm_controller',
             name='arm_controller', output='screen',
             parameters=[{
                 'fruit_registry_path': registry_path,
             }]),
    ])

    # 5. Coordinator (last -- drives the whole FSM)
    coordinator_node = TimerAction(period=15.0, actions=[
        Node(package='greenhouse_robot', executable='robot_coordinator',
             name='robot_coordinator', output='screen'),
    ])

    return LaunchDescription([
        skip_gazebo_arg,
        skip_moveit_arg,
        gazebo_launch,
        moveit_launch,
        perception_nodes,
        arm_node,
        coordinator_node,
    ])

