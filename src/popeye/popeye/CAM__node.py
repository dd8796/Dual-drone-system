#!/usr/bin/env python3
"""
Camera Node for POPEYE Drone.

This node manages video capture from camera or video file and publishes
frames to the ROS2 network for processing by other nodes.
"""

# Import standard utils
from popeye.PARAMS_utils import *
# ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# OpenCV utils
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


#####################################################################################################
##### Camera Node #####
#####################################################################################################
class CAMNode(Node):
    """Node for capturing and publishing camera frames."""

    def __init__(self):
        """Initialize the camera node."""
        super().__init__('CAM_node', namespace='POPEYE')

        ### ROS2 callbacks
        ## Publishers
        self.pub__video_frames = self.create_publisher(
            Image, 'image_raw', 10,
            callback_group=MutuallyExclusiveCallbackGroup())
        
        ## Timers
        self.timer__read_frames = self.create_timer(
            0.05,  # 20 Hz
            self.timer_cb__read_frames,
            callback_group=MutuallyExclusiveCallbackGroup())

        ### START VIDEO CAPTURE
        self.cv_bridge = CvBridge()
        self.vd_capture = None
        self.camera_opened = False
        self.frame_count = 0
        self.error_count = 0
        self.max_consecutive_errors = 10
        
        # Try to initialize camera/video
        self._initialize_video_source()

    def _initialize_video_source(self):
        """Initialize video capture from camera or file."""
        try:
            if use_camera:
                self.get_logger().info("Attempting to open camera...")
                self.vd_capture = cv2.VideoCapture(0)
                
                # Try to set camera properties for better performance
                self.vd_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.vd_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                self.vd_capture.set(cv2.CAP_PROP_FPS, 30)
                
            else:
                # Select video file based on test scenario
                video_files = {
                    'offset': path_DUAV + "/src/popeye/popeye/videos/test_offset.mp4",
                    'offset_diag': path_DUAV + "/src/popeye/popeye/videos/test_offset_diag.mp4",
                    'precision': path_DUAV + "/src/popeye/popeye/videos/precision_landing.mp4",
                    'precision_straight': path_DUAV + "/src/popeye/popeye/videos/precision_landing_straight.mp4",
                }
                
                # Default video file
                video_path = video_files['precision_straight']
                self.get_logger().info(f"Attempting to open video file: {video_path}")
                self.vd_capture = cv2.VideoCapture(video_path)

            # Verify that the capture was opened successfully
            if self.vd_capture is None:
                raise RuntimeError("VideoCapture object is None")
                
            if not self.vd_capture.isOpened():
                raise RuntimeError("Failed to open video source")

            # Test reading a frame
            ret, test_frame = self.vd_capture.read()
            if not ret or test_frame is None:
                raise RuntimeError("Failed to read test frame from video source")

            self.camera_opened = True
            self.get_logger().info(f"✅ Video source opened successfully "
                                 f"(size: {int(self.vd_capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                                 f"{int(self.vd_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))})")
            self.get_logger().info("NODE CAM_node STARTED.")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize video source: {e}")
            self.camera_opened = False
            if self.vd_capture is not None:
                self.vd_capture.release()
                self.vd_capture = None

    ############################################################################################
    ##### TIMER CALLBACK #####
    ############################################################################################
    
    def timer_cb__read_frames(self):
        """
        Timer callback to read and publish video frames.
        
        This function is called at regular intervals to capture and publish frames.
        """
        # Check if camera is opened
        if not self.camera_opened or self.vd_capture is None:
            if self.frame_count % 100 == 0:  # Log every 100 attempts
                self.get_logger().warn("Camera not available, attempting to reinitialize...")
                self._initialize_video_source()
            return

        # Try to read a frame
        try:
            capture_success, frame = self.vd_capture.read()
            
            if not capture_success or frame is None:
                self.error_count += 1
                
                if self.error_count >= self.max_consecutive_errors:
                    self.get_logger().error(
                        f"Failed to capture frame {self.max_consecutive_errors} times, "
                        "reinitializing video source...")
                    self.camera_opened = False
                    if self.vd_capture is not None:
                        self.vd_capture.release()
                    self._initialize_video_source()
                    self.error_count = 0
                return

            # Successfully captured a frame
            self.error_count = 0
            self.frame_count += 1

            # Convert and publish
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub__video_frames.publish(img_msg)

            # Log periodically (every 100 frames to avoid spam)
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Published frame {self.frame_count}')

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")
            self.error_count += 1

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        if self.vd_capture is not None:
            self.vd_capture.release()
            self.get_logger().info("Video capture released.")
        super().destroy_node()


#########################################################################
##### Node entry point #####
#########################################################################
def main(args=None):
    """Main function to run the camera node."""
    rclpy.init(args=args)

    ### Creating the single-thread executor
    node = CAMNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("CAM node spinning...")
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
