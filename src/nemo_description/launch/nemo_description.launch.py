import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import shutil

def generate_launch_description():
    # Direct path to your xacro file
    xacro_path = '/home/zappington/submarine_ws/src/nemo_description/urdf/nemo.urdf.xacro'  # Replace with the actual absolute path    # Run xacro to generate robot description
    robot_description_content = Command(['xacro ', xacro_path])
    robot_description = {'robot_description': robot_description_content}

    # Optional: RViz config path (if you have one)
    rviz_config_path = '/home/zappington/submarine_ws/src/nemo_description/rviz/robot_config.rviz'  # Optional, update path if needed

    return LaunchDescription([
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),
        # RViz (optional config file)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
