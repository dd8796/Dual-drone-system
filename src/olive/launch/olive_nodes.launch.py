#!/usr/bin/env python3
"""
Launch file for OLIVE drone system.

This launch file starts all necessary nodes for the OLIVE drone:
- MAV_manager: MAVLink communication
- analyse_CAM: Camera analysis and fire detection
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate the launch description for OLIVE system."""
    
    # Declare launch arguments
    mavlink_connection_arg = DeclareLaunchArgument(
        'mavlink_connection',
        default_value='tcp:127.0.0.1:5770',
        description='MAVLink connection string (tcp:IP:PORT or udp:IP:PORT or /dev/ttyXXX)'
    )
    
    # Get launch configurations
    mavlink_connection = LaunchConfiguration('mavlink_connection')
    
    # MAV Manager Node
    mav_manager_node = Node(
        package='olive',
        executable='MAV_manager',
        name='MAV_manager',
        namespace='OLIVE',
        output='screen',
        parameters=[{
            'mavlink_connection': mavlink_connection,
        }],
        emulate_tty=True,
    )
    
    # Camera Analysis Node
    analyse_cam_node = Node(
        package='olive',
        executable='analyse_CAM',
        name='analyse_CAM',
        namespace='OLIVE',
        output='screen',
        emulate_tty=True,
    )
    
    # Log info
    launch_info = LogInfo(
        msg=[
            '\n',
            '═══════════════════════════════════════════════════════════\n',
            '  🚁 OLIVE DRONE SYSTEM LAUNCH\n',
            '═══════════════════════════════════════════════════════════\n',
            '  Namespace: OLIVE\n',
            '  Nodes:\n',
            '    - MAV_manager (MAVLink communication)\n',
            '    - analyse_CAM (Fire detection)\n',
            '═══════════════════════════════════════════════════════════\n'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        mavlink_connection_arg,
        
        # Log
        launch_info,
        
        # Nodes
        mav_manager_node,
        analyse_cam_node,
    ])
