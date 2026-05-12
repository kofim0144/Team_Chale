import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    pkg = get_package_share_directory("chaleBOT_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg, "urdf", "chaleBOT.xacro"),
        description="Absolute path to robot urdf file",
    )

    world_file = os.path.join(pkg, "worlds", "greenhouse.world")

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join([
            str(Path(pkg).parent.resolve()),
            pkg,
            os.path.join(pkg, "worlds"),
            os.path.join(pkg, "worlds", "models"),
        ]),
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "chaleBOT",
                   "-x", "-2.5", "-y", "0", "-z", "0.02"],
    )

    # -------- BRIDGE: correct way to pass a yaml config file --------
    bridge_params = os.path.join(pkg, "parameters", "bridge_parameters.yaml")
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
        output="screen",
    )

    # -------- Camera optical frame (REP 103) --------
    # Gazebo publishes images with the header.frame_id set to the link
    # (camera_1), whose axes are X-forward/Y-left/Z-up (URDF convention).
    # ROS image processing, tf2_geometry_msgs, PointCloud2 back-projection,
    # etc. expect the optical frame: Z-forward/X-right/Y-down.
    # This static transform bridges the two so the fruit_detector's
    # transform from camera_1_optical into base_link gives correct XYZ.
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_optical_tf",
        arguments=[
            "--x", "0.1525", "--y", "0.0", "--z", "-0.1",
            "--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5",
            "--frame-id", "camera_1",
            "--child-frame-id", "camera_1_optical",
        ],
    )


    # -------- Bridge Gazebo's scoped frame to RSP's un-scoped frame --------
    # Gazebo's DiffDrive plugin publishes odom -> chaleBOT/base_link (scoped
    # with the model name), but robot_state_publisher publishes base_link and
    # all descendants without a prefix. This static transform unifies the two
    # sides of the TF tree.
    base_link_alias_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_alias_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "chaleBOT/base_link",
            "--child-frame-id", "base_link",
        ],
    )

    # Bridge the scoped DiffDrive frame to the unscoped RSP frame.
    # DiffDrive publishes odom -> chaleBOT/base_footprint
    # RSP publishes the static base_footprint -> base_link chain
    # This static TF connects them.
    base_footprint_alias_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_footprint_alias_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "chaleBOT/base_footprint",
            "--child-frame-id", "base_footprint",
        ],
    )

    # -------- ros2_control controllers spawner --------
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "controllers.launch.py")),
    )

#     wheel_state_relay = Node(
#     package="topic_tools",
#     executable="relay",
#     arguments=["/wheel_states", "/joint_states"],
#     output="screen",
# )


    return LaunchDescription([
        model_arg,
        gz_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        camera_optical_tf,
        # wheel_state_relay,
        # base_link_alias_tf,
        base_footprint_alias_tf,
        controllers_launch
    ])
