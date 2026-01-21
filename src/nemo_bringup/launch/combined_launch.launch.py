from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

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

    # ------------------ Gazebo ------------------
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    nemo_share = get_package_share_directory('nemo_description')

    # ------------------ Robot description ------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_path]),
                value_type=str
            ),
            'use_sim_time': True
        }],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # ------------------ Spawn robot ------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_nemo',
        arguments=[
            '-name', 'nemo_auv',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '-1.0',
            '-world', 'ocean'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    spawn_after_gazebo = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[
                TimerAction(
                    period=2.0,   # ⬅️ REQUIRED for Harmonic
                    actions=[spawn_robot]
                )
            ]
        )
    )

    # ------------------ Bridge ------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
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
            '/IMU/VectorNav1@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    joystick = Node(
        package='joystick_pacs',
        executable='joy_node'
    )

    controller = Node(
        package='nemo_controller',
        executable='simController'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=nemo_share + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_after_gazebo,
        bridge,
        rviz
    ])