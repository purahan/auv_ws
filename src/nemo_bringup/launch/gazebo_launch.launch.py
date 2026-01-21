from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_pkg = 'nemo_description'
    bringup_pac = "nemo_bringup"

    xacro_path = os.path.join(
        get_package_share_directory('nemo_description'),
        'urdf',
        'nemo.urdf.xacro'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('nemo_description'),
        'rviz',
        'robot_config.rviz'
    )

    world_path = os.path.join(
    get_package_share_directory('nemo_bringup'),
        'world',
        'qualification.sdf'
    )

 

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'gz_sim', '-r', world_path],
            output='screen'
        ),

        # 2. robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', xacro_path]),value_type =str),
                'use_sim_time': True
            }],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),

        # 3. Spawn the robot in Gazebo

        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_nemo',
            arguments=[
                '-name', 'nemo_auv',
                '-x', '0.0', '-y', '0.0', '-z', '-1.0',
                '-topic', '/robot_description',
                '-world', 'ocean'
            ],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 4. Gazebo ↔ ROS bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_node',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/nemo_auv/thruster1/cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/nemo_auv/thruster2/cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/nemo_auv/thruster3/cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/nemo_auv/thruster4/cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/nemo_auv/thruster5/cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/nemo_auv/thruster6/cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/nemo_auv/thruster7/cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/nemo_auv/thruster8/cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/nemo_auv/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                'IMU/VectorNav1@sensor_msgs/msg/Imu[gz.msgs.IMU'
            ]
        ),

        # 5. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        #joystick node
        Node(
            package='joystick_pacs',
            executable='joy_node',
            name='joy_node'
        ),

        Node(
            package='nemo_controller',
            executable='simController',
            name='simController'
        )
    ]) 

#,'/VectorNav1@sensor_msgs/msg/Imu[VectorNav1@gz.msgs.IMU',
