#!/usr/bin/env python3
"""
Launch RViz and the robot_state_publisher for visualizing the robot model.

This launch file is responsible for visualizing the robot's state. It starts:
1. robot_state_publisher: To publish TF transforms from the URDF and joint states.
2. joint_state_publisher_gui: A GUI to manually control robot joints (optional).
3. RViz: The visualization tool, loaded with a specific configuration.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():


    # ========================= Paths & Environment Setup =========================== #   
    
    # 1. Get the paths to the required packages
    pkg_gripperbot_description = get_package_share_directory('gripperbot_description')

    # 2. Define file paths
    xacro_file = os.path.join(pkg_gripperbot_description, 'urdf', 'gripperbot.urdf.xacro')


    # ================== Declare Launch Arguments =================== #

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Use simulation (Gazebo) clock if true. Leave false when no /clock source is available.'
    )

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        'jsp_gui', 
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
        
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([pkg_gripperbot_description, 'rviz', 'display_only_config.rviz']),
        description='Full path to the RViz config file.'
    )

    declare_sim_not_real = DeclareLaunchArgument(
        'sim_not_real',
        default_value='true', 
        description='Whether we are running in simulation (true) or on hardware (false).'
    )
    

    # ================== Robot Description Setup =================== #

    #Process URDF (to pass any xacro arguments through)
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file, ' ', 'sim_not_real:=', LaunchConfiguration('sim_not_real')]),
        value_type=str
    )

    # ================== Node Definitions =================== #


    # ---- Robot State Publisher ----#
    
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content,
        }]
        # No remappings here
    )


    # --------- Joint State Publisher GUI Node ----------

    start_joint_state_publisher_gui_cmd = Node( # Starts JSP and the GUI if jsp_gui is true
        condition=IfCondition(LaunchConfiguration('jsp_gui')),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        
    )

    start_joint_state_publisher_cmd = Node( # Starts JSP without GUI if jsp_gui is false
        condition=UnlessCondition(LaunchConfiguration('jsp_gui')),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ---- RViz2 Node ----

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        # No remappings here
    )

    # ================== Create Launch Description =================== #

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_sim_not_real)

    # Add nodes to the launch description
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld