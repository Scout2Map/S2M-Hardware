import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('s2m_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'scout2map.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')

    # xacro is expanded here so robot_description is a plain URDF string
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # requires a Gazebo world already running, e.g. via scout_sim_bringup
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'scout2map',
            '-x', x_pose, '-y', y_pose, '-z', '0.05',
        ],
        output='screen',
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    return LaunchDescription([
        declare_x,
        declare_y,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
    ])
