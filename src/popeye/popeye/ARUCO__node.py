#!/usr/bin/env python3
"""
ARUCO Detection Node for POPEYE Drone.

This node detects ArUco markers from the camera feed and calculates
their GPS positions based on the drone's position and altitude.
"""

# Import standard utils
import numpy as np
import math
from time import sleep
from popeye.PARAMS_utils import *
# ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# OpenCV utils
from interfaces.msg import GpsPosition, Attitude
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


#####################################################################################################
##### ARUCO Detection Node #####
#####################################################################################################
class ARUCONode(Node):
    """Node for detecting ArUco markers and calculating their GPS positions."""

    def __init__(self):
        """Initialize the ARUCO detection node."""
        super().__init__('ARUCO_node', namespace='POPEYE')

        ### ROS2 Callbacks
        ## Callback groups
        non_critical_group = ReentrantCallbackGroup()
        
        ## Subscribers
        self.create_subscription(
            Image, 'image_raw', self.sub_cb__image_raw, 10,
            callback_group=non_critical_group)
        self.create_subscription(
            GpsPosition, 'uav_position', self.sub_cb__uav_position, 10,
            callback_group=non_critical_group)
        self.create_subscription(
            Attitude, 'uav_attitude', self.sub_cb__uav_attitude, 10,
            callback_group=non_critical_group)
        
        ## Publishers
        self.pub__cam_fire_pos = self.create_publisher(
            GpsPosition, 'CAM/fire_pos', 10, callback_group=non_critical_group)
        self.pub__cam_park_pos = self.create_publisher(
            GpsPosition, 'CAM/park_pos', 10, callback_group=non_critical_group)
        self.pub__img_debug = self.create_publisher(
            Image, 'image_debug', 10, callback_group=non_critical_group)

        ### GLOBAL PARAMS
        self.cv_bridge = CvBridge()
        fov = math.radians(100)
        self.img_center = np.array([1280/2, 720/2])
        self.constant_pixel_to_meters = 2*math.tan(fov/2) / 1280 / 1.27  # Corrected constant
        
        # Initialize all state variables to None for proper error handling
        self.uav_position = None
        self.uav_alt = None
        self.uav_roll = None
        self.uav_pitch = None
        
        # Statistics for debugging
        self.frame_count = 0
        self.detection_count = 0
        
        self.get_logger().info("ARUCO_node started successfully.")

    ############################################################################################
    ##### SUBSCRIBERS CALLBACKS #####
    ############################################################################################
    
    def sub_cb__uav_position(self, msg):
        """
        Callback for UAV position updates.
        
        Args:
            msg: GpsPosition message containing lat, lon, alt
        """
        if not is_valid_gps(msg.lat, msg.lon):
            self.get_logger().warn(f"Invalid GPS position received: ({msg.lat}, {msg.lon})")
            return
            
        self.uav_position = (msg.lat, msg.lon)
        self.uav_alt = msg.alt
        
    def sub_cb__uav_attitude(self, msg):
        """
        Callback for UAV attitude updates.
        
        Args:
            msg: Attitude message containing roll, pitch, yaw
        """
        self.uav_roll = msg.roll
        self.uav_pitch = msg.pitch
        
    def sub_cb__image_raw(self, msg):
        """
        Callback for processing incoming video frames.
        
        Args:
            msg: Image message from the camera
        """
        self.frame_count += 1
        
        ## Verify that we received all needed variables
        if self.uav_position is None or self.uav_alt is None:
            if self.frame_count % 50 == 0:  # Log only every 50 frames to avoid spam
                self.get_logger().warn("Waiting for UAV position data...")
            return
            
        if self.uav_roll is None or self.uav_pitch is None:
            if self.frame_count % 50 == 0:
                self.get_logger().warn("Waiting for UAV attitude data...")
            return

        ## Don't try to use camera if the drone is tilted
        if uav_is_tilted(self.uav_roll, self.uav_pitch):
            if debug_cams:
                self.get_logger().warn(
                    f"UAV is tilted (roll: {self.uav_roll*TO_DEG:.1f}°, "
                    f"pitch: {self.uav_pitch*TO_DEG:.1f}°), skipping frame.")
            return

        ## Save current state (prevents race conditions)
        at_time_uav_position = self.uav_position
        at_time_uav_alt = self.uav_alt

        ## Get current frame
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        if frame is None or frame.size == 0:
            self.get_logger().warn("Empty frame received from camera.")
            return

        ## Determine target centers
        aruco_centers = self.find_aruco_centers(frame)

        ## Compute offsets and publish positions
        if aruco_centers is not None:
            for aruco_center in aruco_centers:
                aruco_id = aruco_center[0]
                center = aruco_center[1]

                ## Check if id is recognized
                if aruco_id not in [5, 222]:
                    self.get_logger().warn(f"Unknown ARUCO tag ID: {aruco_id}")
                    continue

                ## Compute offset using ACTUAL altitude (BUG FIX: was using 1 instead)
                offset = self.offset_to_meters(at_time_uav_alt, center - self.img_center)
                dist_to_target = np.linalg.norm(offset)

                ## Compute the target GPS position
                target_lat, target_lon = self.offset_to_target_position(
                    at_time_uav_position, offset)
                
                # Validate computed GPS coordinates
                if not is_valid_gps(target_lat, target_lon):
                    self.get_logger().error(
                        f"Computed invalid GPS position: ({target_lat}, {target_lon})")
                    continue

                ## Prepare message
                msg_pub = GpsPosition()
                msg_pub.lat = float(target_lat)
                msg_pub.lon = float(target_lon)
                msg_pub.alt = float(0.)

                ## Publish based on marker ID
                if aruco_id == 5:
                    self.get_logger().info(
                        f"PARK target detected at distance {dist_to_target:.2f}m")
                    self.pub__cam_park_pos.publish(msg_pub)
                    self.detection_count += 1
                elif aruco_id == 222:
                    self.get_logger().info(
                        f"FIRE target detected at distance {dist_to_target:.2f}m")
                    self.pub__cam_fire_pos.publish(msg_pub)
                    self.detection_count += 1

                ## Debug visualization
                if debug_cams:
                    self.get_logger().info(
                        f"Offset (m): ({offset[0]:.3f}, {offset[1]:.3f}) "
                        f"=> distance (m): {dist_to_target:.2f}")
                    cv2.line(frame,
                            (int(self.img_center[0]), int(self.img_center[1])),
                            (int(center[0]), int(center[1])),
                            (0, 255, 0), 2)

        ## Publish debug images
        if debug_cams:
            try:
                self.pub__img_debug.publish(self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))
            except Exception as e:
                self.get_logger().error(f"Failed to publish debug image: {e}")

    ############################################################################################
    ##### TOOLS #####
    ############################################################################################
    
    def offset_to_meters(self, altitude: float, offset_pixels: np.ndarray) -> np.ndarray:
        """
        Convert pixel offset to meters using camera specifications and drone altitude.
        
        Args:
            altitude: Drone altitude in meters
            offset_pixels: Pixel offset from image center [x, y]
        
        Returns:
            np.ndarray: Offset in meters [east_m, north_m]
        """
        if altitude <= 0:
            self.get_logger().error(f"Invalid altitude: {altitude}m")
            return np.array([0.0, 0.0])
            
        offset_m = np.array(offset_pixels) * altitude * self.constant_pixel_to_meters
        return offset_m
        
    def offset_to_target_position(self, uav_position: tuple, offset: np.ndarray) -> tuple:
        """
        Compute target GPS position from offset in meters.
        
        Args:
            uav_position: Current UAV position (lat, lon)
            offset: Offset in meters [east, north]
        
        Returns:
            tuple: Target position (lat, lon)
        
        Reference:
            https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters
        """
        # offset[0] = east-west (x in image), offset[1] = north-south (y in image)
        # Note: In image coordinates, y increases downward, so we negate it
        dx_meters = offset[0]  # East-West
        dy_meters = -offset[1]  # North-South (negated because image y is inverted)
        
        # Calculate new position
        target_lat = uav_position[0] + dy_meters * CONV
        target_lon = uav_position[1] + dx_meters * CONV / np.cos(target_lat * TO_RAD)
        
        return (target_lat, target_lon)
        
    def find_aruco_centers(self, frame: np.ndarray) -> list:
        """
        Find ArUco markers in frame and return their centers.
        
        Args:
            frame: Input image frame
        
        Returns:
            list: List of [marker_id, center_position] for each detected marker,
                  or None if no markers detected
        """
        ## Select ArUco dictionary and parameters
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = cv2.aruco.DetectorParameters()

        ## Find the aruco markers (convert to grayscale for better detection)
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            marker_corners, marker_ids, rejected = detector.detectMarkers(gray)
        except Exception as e:
            self.get_logger().error(f"ArUco detection failed: {e}")
            return None

        if marker_ids is None or len(marker_ids) == 0:
            return None

        ## Calculate centers
        aruco_centers = []
        for i, marker_id in enumerate(marker_ids):
            corner = marker_corners[i][0]
            center_x = int((corner[0][0] + corner[2][0]) / 2)
            center_y = int((corner[0][1] + corner[2][1]) / 2)
            aruco_centers.append([int(marker_id[0]), np.array([center_x, center_y])])

            if debug_cams:
                # Draw center point
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                # Draw marker ID
                cv2.putText(frame, f"ID:{int(marker_id[0])}",
                           (center_x + 10, center_y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if debug_cams:
            cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

        return aruco_centers


#########################################################################
##### Node entry point #####
#########################################################################
def main(args=None):
    """Main function to run the ARUCO node."""
    rclpy.init(args=args)

    ### Creating the multithread executor
    node = ARUCONode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    
    try:
        node.get_logger().info("ARUCO node spinning...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.get_logger().info("Destroying node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
