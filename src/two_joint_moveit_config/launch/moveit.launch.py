import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory('two_joint_description')
    config_share = get_package_share_directory('two_joint_moveit_config')
    bringup_share = get_package_share_directory('two_joint_bringup')

    xacro_file = os.path.join(description_share, 'urdf', 'two_joint_robot.urdf.xacro')
    controllers_file = os.path.join(bringup_share, 'config', 'controllers.yaml')

    moveit_config = (
        MoveItConfigsBuilder('two_joint_robot', package_name='two_joint_moveit_config')
        .robot_description(
            file_path=xacro_file,
            mappings={'controllers_file': controllers_file}
        )
        .robot_description_semantic(file_path=os.path.join(config_share, 'config', 'two_joint_robot.srdf'))
        .robot_description_kinematics(file_path=os.path.join(config_share, 'config', 'kinematics.yaml'))
        .joint_limits(file_path=os.path.join(config_share, 'config', 'joint_limits.yaml'))
        .trajectory_execution(file_path=os.path.join(config_share, 'config', 'moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='screen',
        arguments=['-d', os.path.join(config_share, 'config', 'moveit.rviz')],
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}]
    )

    return LaunchDescription([move_group, rviz])
