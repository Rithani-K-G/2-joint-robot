import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('two_joint_description')
    bringup_share = get_package_share_directory('two_joint_bringup')
    xacro_file = os.path.join(description_share, 'urdf', 'two_joint_robot.urdf.xacro')
    controllers_file = os.path.join(bringup_share, 'config', 'controllers.yaml')

    robot_description = Command([
        'xacro ', xacro_file,
        ' controllers_file:=', controllers_file
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        )
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'two_joint_robot'],
        output='screen'
    )

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        TimerAction(period=5.0, actions=[jsb]),
        TimerAction(period=7.0, actions=[arm]),
    ])
