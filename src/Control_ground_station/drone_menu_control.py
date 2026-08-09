import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import CommandHome
from mavros_msgs.srv import WaypointClear
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import HomePosition
import os

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.waypoint_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.set_home_client = self.create_client(CommandHome, '/mavros/cmd/set_home')
        self.home_position = None

        # Subscribe to home position
        self.create_subscription(HomePosition, '/mavros/home_position/home', self.home_position_callback, 10)

        # Wait for the arming service to be available
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')

        # Wait for the waypoint push service to be available
        while not self.waypoint_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for waypoint push service...')

        # Wait for the waypoint clear service to be available
        while not self.waypoint_clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for waypoint clear service...')

        # Wait for the set home service to be available
        while not self.set_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set home service...')

        self.get_logger().info('Services ready!')

    def home_position_callback(self, msg):
        self.get_logger().info(f"Raw home position message received: {msg}")
        if not (msg.geo.latitude and msg.geo.longitude and msg.geo.altitude):
            self.get_logger().error("Home position data is incomplete.")
            return

        self.home_position = msg.geo
        self.get_logger().info(f"Processed home position: Lat: {msg.geo.latitude}, Lon: {msg.geo.longitude}, Alt: {msg.geo.altitude}")

    def arm_drone(self, arm: bool):
        request = CommandBool.Request()
        request.value = arm
        future = self.arming_client.call_async(request)
        
        # Wait for the result
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                action = "armed" if arm else "disarmed"
                self.get_logger().info(f'Drone successfully {action}.')
            else:
                self.get_logger().error(f'Failed to {"arm" if arm else "disarm"} the drone.')
        else:
            self.get_logger().error('Service call failed.')

    def upload_waypoints(self, file_path):
        if not os.path.exists(file_path):
            self.get_logger().error(f'File not found: {file_path}')
            return

        # Read waypoints from file
        waypoints = []
        try:
            with open(file_path, 'r') as file:
                header = file.readline().strip()
                if header != "QGC WPL 110":
                    self.get_logger().error("Invalid waypoint file format. Expected 'QGC WPL 110'.")
                    return

                for line in file.readlines():
                    line = line.strip()
                    if not line or line.startswith('#'):  # Skip empty lines or comments
                        continue

                    waypoint_data = line.split('\t')
                    if len(waypoint_data) < 12:  # Ensure there are enough fields
                        self.get_logger().error(f"Invalid waypoint data: {line}")
                        continue

                    waypoint = Waypoint(
                        frame=int(waypoint_data[2]),  # MAV_FRAME
                        command=int(waypoint_data[3]),  # MAV_CMD
                        is_current=bool(int(waypoint_data[1])),  # Current waypoint
                        autocontinue=bool(int(waypoint_data[11])),  # Autocontinue
                        param1=float(waypoint_data[4]),  # Param1
                        param2=float(waypoint_data[5]),  # Param2
                        param3=float(waypoint_data[6]),  # Param3
                        param4=float(waypoint_data[7]),  # Param4
                        x_lat=float(waypoint_data[8]),  # Latitude
                        y_long=float(waypoint_data[9]),  # Longitude
                        z_alt=float(waypoint_data[10])  # Altitude
                    )
                    waypoints.append(waypoint)

        except Exception as e:
            self.get_logger().error(f'Failed to read waypoints: {e}')
            return

        if not waypoints:
            self.get_logger().error("No valid waypoints found in the file.")
            return

        request = WaypointPush.Request()
        request.waypoints = waypoints
        future = self.waypoint_push_client.call_async(request)
        
        # Wait for the result
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Successfully uploaded {len(waypoints)} waypoints.')
            else:
                self.get_logger().error('Failed to upload waypoints.')
        else:
            self.get_logger().error('Service call failed.')


    def clear_waypoints(self):
        request = WaypointClear.Request()
        future = self.waypoint_clear_client.call_async(request)
        
        # Wait for the result
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info('All waypoints successfully cleared.')
        else:
            self.get_logger().error('Failed to clear waypoints.')

    def send_simple_waypoint(self):
        if self.home_position is None:
            self.get_logger().info('Waiting for home position...')
            timeout = 10  # Wait for 10 seconds
            while self.home_position is None and timeout > 0 and rclpy.ok():
                rclpy.spin_once(self)
                timeout -= 1

        if self.home_position is None:  # Final check
            self.get_logger().error('Home position not available. Cannot send waypoints.')
            return

        # Use home position as base
        home_lat = self.home_position.latitude
        home_lon = self.home_position.longitude
        home_alt = self.home_position.altitude

        # Create waypoints near the home position
        waypoint1 = Waypoint(
            frame=3,  # GLOBAL
            command=16,  # NAV_WAYPOINT
            is_current=True,
            autocontinue=True,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            x_lat=home_lat,
            y_long=home_lon,
            z_alt=home_alt + 10.0
        )

        waypoint2 = Waypoint(
            frame=3,  # GLOBAL
            command=16,  # NAV_WAYPOINT
            is_current=False,
            autocontinue=True,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            x_lat=home_lat + 0.0001,  # Slightly north
            y_long=home_lon,
            z_alt=home_alt + 15.0
        )

        waypoint3 = Waypoint(
            frame=3,  # GLOBAL
            command=16,  # NAV_WAYPOINT
            is_current=False,
            autocontinue=True,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            x_lat=home_lat,
            y_long=home_lon + 0.0001,  # Slightly east
            z_alt=home_alt + 20.0
        )

        request = WaypointPush.Request()
        request.waypoints = [waypoint1, waypoint2, waypoint3]

        future = self.waypoint_push_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Successfully sent a simple waypoint plan.')
        else:
            self.get_logger().error('Failed to send the simple waypoint plan.')

    def set_home_position(self, latitude, longitude, altitude):
        request = CommandHome.Request()
        request.latitude = latitude
        request.longitude = longitude
        request.altitude = altitude
        request.current_gps = False

        future = self.set_home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Home position successfully set.')
        else:
            self.get_logger().error('Failed to set home position.')

    def display_home_position(self):
        if self.home_position is None:
            self.get_logger().info('Home position not available yet.')
        else:
            self.get_logger().info(f"Home position: Lat: {self.home_position.latitude}, Lon: {self.home_position.longitude}, Alt: {self.home_position.altitude}")

def main():
    rclpy.init()
    node = DroneControlNode()

    try:
        while rclpy.ok():
            print("\nMenu:")
            print("1. Arm the drone")
            print("2. Disarm the drone")
            print("3. Upload waypoints")
            print("4. Send a simple waypoint")
            print("5. Display home position")
            print("6. Set home position")
            print("   1. Enter custom home position")
            print("   2. Set home position to (0, 0, 0)")
            print("   3. Set home position to (44.80164096922527, -0.6036625300698428, 0)")
            print("7. Clear all waypoints")
            print("8. Exit")
            user_input = input("Enter your choice: ")

            if user_input == "1":
                node.arm_drone(True)
            elif user_input == "2":
                node.arm_drone(False)
            elif user_input == "3":
                file_path = input("Enter the path to the .waypoint file: ")
                node.upload_waypoints(file_path)
            elif user_input == "4":
                node.send_simple_waypoint()
            elif user_input == "5":
                node.display_home_position()
            elif user_input == "6":
                print("\nSet Home Position:")
                print("1. Enter custom home position")
                print("2. Set home position to (0, 0, 0)")
                print("3. Set home position to (44.80164096922527, -0.6036625300698428, 0)")
                home_input = input("Enter your choice: ")
                if home_input == "1":
                    latitude = float(input("Enter latitude: "))
                    longitude = float(input("Enter longitude: "))
                    altitude = float(input("Enter altitude: "))
                    node.set_home_position(latitude, longitude, altitude)
                elif home_input == "2":
                    node.set_home_position(0.0, 0.0, 0.0)
                elif home_input == "3":
                    node.set_home_position(44.80164096922527, -0.6036625300698428, 0.0)
                else:
                    print("Invalid input. Returning to main menu.")
            elif user_input == "7":
                node.clear_waypoints()
            elif user_input == "8":
                print("Exiting...")
                break
            else:
                print("Invalid input. Please enter a valid choice.")
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
