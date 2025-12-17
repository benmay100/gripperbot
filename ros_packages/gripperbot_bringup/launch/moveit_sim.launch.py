#!/usr/bin/env python3
"""
Top-level launch file to start MoveIt2 only, and load the robot arm into it.
This launch file triggers the following lower level launch files:
    File: _________________ | Package: _________________
    File: _________________ | Package: _________________
    File: _________________ | Package: _________________
    
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():


    # ========================= Paths & Environment Setup =========================== #   
    
    # 1. Get the paths to the required packages
    pkg_gripperbot_moveit = get_package_share_directory('gripperbot_moveit')

    controllers_file = os.path.join(pkg_gripperbot_moveit, 'config', 'ros2_controllers.yaml')


    # ========================= Declare Launch Arguments =========================== #   

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([pkg_gripperbot_moveit, 'config', 'moveit_v2.rviz']),
        description='Full path to the RViz config file.'
    )

    # =============================================================================== # 



    # ================== Launch MoveIt Configuration Builder Here =================== #

    moveit_config = (
        MoveItConfigsBuilder('gripperbot', package_name='gripperbot_moveit')
        .robot_description()
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl', 'stomp', 'pilz_industrial_motion_planner'])
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    # =============================================================================== # 



    # ============================= Launch MoveIt2 Here ============================== #

    start_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # =================================================================================#

    # ======================== Launch RVIZ & Robot State Publisher ========================= # 
    
    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ================================================================================ # 



    # ======================= Launch ROS2 Control Nodes here ========================= #

    start_ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            controllers_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delayed_joint_state_spawner = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_controller_spawners = TimerAction(
        period=4.0,
        actions=[arm_controller_spawner, gripper_controller_spawner],
    )

    # ================================================================================ # 
    

    # ========================= Create Launch Description ============================ # 
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_rviz_node)
    ld.add_action(start_move_group_node)
    ld.add_action(start_ros2_control_node)
    ld.add_action(delayed_joint_state_spawner)
    ld.add_action(delayed_controller_spawners)

    return ld