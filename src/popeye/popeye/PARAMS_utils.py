#!/usr/bin/env python3
"""
Utility parameters and functions for the POPEYE drone system.

This module contains global constants, conversion utilities, and helper functions
used across the drone control system.
"""

import math

# Colors for console print
RESET = "\033[0m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"
BOLD = "\033[1m"
UNDERLINE = "\033[4m"
REVERSED = "\033[7m"

# Home positions
SGDB_LAT = 48.6126523
SGDB_LON = 2.3963258
DUAV_LAT = 45.4389466
DUAV_LON = -0.4283327
DEFAULT_LAT = SGDB_LAT
DEFAULT_LON = SGDB_LON
DEFAULT_ALT = 6

# Global parameters
PI = math.pi
EARTH_RAD = 6371000  # Earth radius in meters
TO_RAD = PI / 180    # Degrees to radians conversion
TO_DEG = 180 / PI    # Radians to degrees conversion
CONV = TO_RAD / EARTH_RAD  # Conversion factor for GPS offset calculations

# Debug and configuration
on_raspi = False
debug_cams = True
use_camera = False
if on_raspi:
    debug_cams = False

# Path configurations
path_step = "/home/step/ros2_DUAV"
path_felix = "/home/linux/ros2_DUAV"
path_popeye = "/home/popeye/ros2_DUAV"
path_DUAV = path_step
if on_raspi:
    path_DUAV = path_popeye

# Tilt detection parameters
MAX_ROLL_ANGLE_DEG = 5.0   # Maximum acceptable roll angle in degrees
MAX_PITCH_ANGLE_DEG = 5.0  # Maximum acceptable pitch angle in degrees


############################################################################################
##### CONVERSION UTILS #####
############################################################################################

def uav_is_tilted(attitude_roll: float, attitude_pitch: float, 
                  max_roll: float = MAX_ROLL_ANGLE_DEG, 
                  max_pitch: float = MAX_PITCH_ANGLE_DEG) -> bool:
    """
    Check if the UAV is tilted beyond acceptable limits.
    
    Args:
        attitude_roll: Roll angle in radians (from MAVLink ATTITUDE message)
        attitude_pitch: Pitch angle in radians (from MAVLink ATTITUDE message)
        max_roll: Maximum acceptable roll angle in degrees (default: 5.0)
        max_pitch: Maximum acceptable pitch angle in degrees (default: 5.0)
    
    Returns:
        bool: True if the UAV is tilted beyond limits, False otherwise
    
    Note:
        MAVLink ATTITUDE message provides angles in radians.
        This function converts to degrees for comparison with human-readable limits.
    """
    # Convert radians to degrees
    roll_deg = abs(attitude_roll * TO_DEG)
    pitch_deg = abs(attitude_pitch * TO_DEG)
    
    # Check if either angle exceeds the limit
    if roll_deg > max_roll or pitch_deg > max_pitch:
        return True
    return False


def is_valid_gps(lat: float, lon: float) -> bool:
    """
    Validate GPS coordinates.
    
    Args:
        lat: Latitude in degrees
        lon: Longitude in degrees
    
    Returns:
        bool: True if coordinates are valid, False otherwise
    """
    # Check latitude bounds (-90 to 90)
    if lat < -90.0 or lat > 90.0:
        return False
    
    # Check longitude bounds (-180 to 180)
    if lon < -180.0 or lon > 180.0:
        return False
    
    # Check if coordinates are not (0, 0) which is often a default invalid value
    if abs(lat) < 0.0001 and abs(lon) < 0.0001:
        return False
    
    return True


def meters_to_gps_offset(dx: float, dy: float, current_lat: float) -> tuple:
    """
    Convert offset in meters to GPS coordinate offset.
    
    Args:
        dx: East-West offset in meters (positive = East)
        dy: North-South offset in meters (positive = North)
        current_lat: Current latitude in degrees
    
    Returns:
        tuple: (delta_lat, delta_lon) in degrees
    """
    delta_lat = dy * CONV
    delta_lon = dx * CONV / math.cos(current_lat * TO_RAD)
    
    return delta_lat, delta_lon


def gps_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate distance between two GPS coordinates using Haversine formula.
    
    Args:
        lat1, lon1: First point coordinates in degrees
        lat2, lon2: Second point coordinates in degrees
    
    Returns:
        float: Distance in meters
    """
    # Convert to radians
    lat1_rad = lat1 * TO_RAD
    lon1_rad = lon1 * TO_RAD
    lat2_rad = lat2 * TO_RAD
    lon2_rad = lon2 * TO_RAD
    
    # Haversine formula
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    
    return EARTH_RAD * c
