#!/usr/bin/env python3
"""
Launch file for POPEYE drone system.

This launch file starts all necessary nodes for the POPEYE drone:
- MAV_manager_node: MAVLink communication
- CAM_node: Camera/video capture
- ARUCO_node: ArUco marker detection
- FSM_node: Finite State Machine control
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate the launch description for POPEYE system."""
    
    # Declare launch arguments
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Use physical camera instead of video file'
    )
    
    debug_cams_arg = DeclareLaunchArgument(
        'debug_cams',
        default_value='true',
        description='Enable camera debug visualization'
    )
    
    on_raspi_arg = DeclareLaunchArgument(
        'on_raspi',
        default_value='false',
        description='Running on Raspberry Pi (changes MAVLink connection and paths)'
    )
    
    # Get launch configurations
    use_camera = LaunchConfiguration('use_camera')
    debug_cams = LaunchConfiguration('debug_cams')
    on_raspi = LaunchConfiguration('on_raspi')
    
    # MAV Manager Node
    mav_manager_node = Node(
        package='popeye',
        executable='MAV_manager_node',
        name='MAV_manager_node',
        namespace='POPEYE',
        output='screen',
        parameters=[{
            'on_raspi': on_raspi,
        }],
        emulate_tty=True,
    )
    
    # Camera Node
    cam_node = Node(
        package='popeye',
        executable='CAM_node',
        name='CAM_node',
        namespace='POPEYE',
        output='screen',
        parameters=[{
            'use_camera': use_camera,
        }],
        emulate_tty=True,
    )
    
    # ARUCO Detection Node
    aruco_node = Node(
        package='popeye',
        executable='ARUCO_node',
        name='ARUCO_node',
        namespace='POPEYE',
        output='screen',
        parameters=[{
            'debug_cams': debug_cams,
        }],
        emulate_tty=True,
    )
    
    # FSM Control Node
    fsm_node = Node(
        package='popeye',
        executable='FSM_node',
        name='FSM_node',
        namespace='POPEYE',
        output='screen',
        parameters=[{
            'on_raspi': on_raspi,
        }],
        emulate_tty=True,
        prefix='xterm -e',  # Run in separate terminal for menu interaction
    )
    
    # Log info
    launch_info = LogInfo(
        msg=[
            '\n',
            '═══════════════════════════════════════════════════════════\n',
            '  🚁 POPEYE DRONE SYSTEM LAUNCH\n',
            '═══════════════════════════════════════════════════════════\n',
            '  Namespace: POPEYE\n',
            '  Nodes:\n',
            '    - MAV_manager_node (MAVLink communication)\n',
            '    - CAM_node (Camera/video capture)\n',
            '    - ARUCO_node (ArUco marker detection)\n',
            '    - FSM_node (Mission control)\n',
            '═══════════════════════════════════════════════════════════\n'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        use_camera_arg,
        debug_cams_arg,
        on_raspi_arg,
        
        # Log
        launch_info,
        
        # Nodes
        mav_manager_node,
        cam_node,
        aruco_node,
        fsm_node,
    ])
