# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_moveit_rviz_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("chaleBOT", package_name="chaleBOT_moveIt_config").to_moveit_configs()
#     return generate_moveit_rviz_launch(moveit_config)



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("chaleBOT", package_name="chaleBOT_moveIt_config")
        .robot_description(file_path="config/chaleBOT.urdf.xacro")
        .robot_description_semantic(file_path="config/chaleBOT.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("chaleBOT_moveIt_config"),
        "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([rviz_node])