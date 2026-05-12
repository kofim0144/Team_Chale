import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    chaleBOT_description_dir = get_package_share_directory("chaleBOT_description")
    urdf_path = os.path.join(chaleBOT_description_dir, "urdf", "chaleBOT.xacro")

    moveit_config = (
        MoveItConfigsBuilder("chaleBOT", package_name="chaleBOT_moveIt_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path="config/chaleBOT.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Trajectory execution tolerances. The 100.0 scaling means MoveIt waits
    # up to 100x the planned trajectory time before declaring timeout. The
    # 60.0 goal duration margin adds 60s of slack at the end.
    trajectory_execution_params = {
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 100.0,
            "allowed_goal_duration_margin": 60.0,
            "allowed_start_tolerance": 0.1,
        }
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            trajectory_execution_params,
            {"use_sim_time": True},
            {"publish_robot_description_semantic": True},
            {"monitor_dynamic_object_pose": False},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("chaleBOT_moveIt_config"),
        "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])