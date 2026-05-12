import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import launch_ros


def generate_launch_description():
    pkg_name = 'chaleBOT_description'
    pkg_share = get_package_share_directory(pkg_name)

    default_model_path = os.path.join(pkg_share, 'urdf', 'chaleBOT.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'launch', 'urdf.rviz')

    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    rvizconfig = LaunchConfiguration('rvizconfig')

    robot_description = Command(['xacro ', model])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to robot xacro file'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to RViz config file'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rvizconfig]
        )
    ])



