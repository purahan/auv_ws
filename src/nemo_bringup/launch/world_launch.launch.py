from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world_path = os.path.join(
    get_package_share_directory('nemo_bringup'),
        'world',
        'qualification.sdf'
    )


    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim','-r', world_path],
            output='screen'
        )
    ]) 
