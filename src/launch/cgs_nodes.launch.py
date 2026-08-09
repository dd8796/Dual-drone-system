#!/usr/bin/env python3
"""
Launch file for Ground Control Station (CGS).

This launch file starts the communication bridge between OLIVE and POPEYE drones:
- MAV_manager_olive: Receives data from OLIVE
- MAV_manager_popeye: Sends data to POPEYE
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate the launch description for CGS system."""
    
    # Declare launch arguments for OLIVE connection
    olive_connection_arg = DeclareLaunchArgument(
        'olive_connection',
        default_value='tcp:127.0.0.1:5773',
        description='MAVLink connection string for OLIVE drone'
    )
    
    # Declare launch arguments for POPEYE connection
    popeye_connection_arg = DeclareLaunchArgument(
        'popeye_connection',
        default_value='tcp:127.0.0.1:5783',
        description='MAVLink connection string for POPEYE drone'
    )
    
    # Get launch configurations
    olive_connection = LaunchConfiguration('olive_connection')
    popeye_connection = LaunchConfiguration('popeye_connection')
    
    # OLIVE MAV Manager Node
    mav_manager_olive_node = Node(
        package='control_ground_station',
        executable='MAV_manager_olive',
        name='MAV_manager_olive',
        namespace='CGS',
        output='screen',
        parameters=[{
            'mavlink_connection': olive_connection,
        }],
        emulate_tty=True,
    )
    
    # POPEYE MAV Manager Node
    mav_manager_popeye_node = Node(
        package='control_ground_station',
        executable='MAV_manager_popeye',
        name='MAV_manager_popeye',
        namespace='CGS',
        output='screen',
        parameters=[{
            'mavlink_connection': popeye_connection,
        }],
        emulate_tty=True,
    )
    
    # Log info
    launch_info = LogInfo(
        msg=[
            '\n',
            '═══════════════════════════════════════════════════════════\n',
            '  🖥️  GROUND CONTROL STATION (CGS) LAUNCH\n',
            '═══════════════════════════════════════════════════════════\n',
            '  Namespace: CGS\n',
            '  Nodes:\n',
            '    - MAV_manager_olive (Receive from OLIVE)\n',
            '    - MAV_manager_popeye (Send to POPEYE)\n',
            '\n',
            '  Communication Flow:\n',
            '    OLIVE → MAV_manager_olive → ROS2 Topics → \n',
            '    → MAV_manager_popeye → POPEYE\n',
            '═══════════════════════════════════════════════════════════\n'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        olive_connection_arg,
        popeye_connection_arg,
        
        # Log
        launch_info,
        
        # Nodes
        mav_manager_olive_node,
        mav_manager_popeye_node,
    ])
