"""
controllers.launch.py
---------------------
Sequentially spawns the controllers defined in controller.yaml.
The controller_manager itself is provided by the gz_ros2_control
plugin loaded inside Gazebo, so this launch only spawns controllers.

Spawn order (each waits for the previous to exit):
  1. joint_state_broadcaster
  2. arm_controller
  3. gripper_controller
  4. camera_controller
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def _spawner(controller_name: str) -> Node:
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller_name,
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )


def generate_launch_description():
    jsb_spawner     = _spawner('joint_state_broadcaster')
    arm_spawner     = _spawner('arm_controller')
    gripper_spawner = _spawner('gripper_controller')
    camera_spawner  = _spawner('camera_controller')

    return LaunchDescription([
        # Delay first spawner so Gazebo + gz_ros2_control have time to
        # start the controller_manager before we try to talk to it.
        TimerAction(period=5.0, actions=[jsb_spawner]),

        # Each subsequent spawner waits for the previous one to exit.
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=jsb_spawner, on_exit=[arm_spawner])),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=arm_spawner, on_exit=[gripper_spawner])),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=gripper_spawner, on_exit=[camera_spawner])),
    ])