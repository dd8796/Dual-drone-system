#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil

class MAVManagerPopeye(Node):
    def __init__(self):
        super().__init__('MAV_manager_popeye', namespace='CGS')

        # Connexion à MAVLink
        try:
            # ADRUPILOT SITL
            self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5783', source_system=1, source_component=2, baud=115200)
            # PX4 SITL
            # self.mavlink_connection = mavutil.mavlink_connection('udp:127.0.0.1:14032', source_system=1, source_component=2)
            # RADIO
            # self.mavlink_connection = mavutil.mavlink_connection('/dev/ttyUSB0', source_system=1, source_component=2, baud=57600)   
        except Exception as e:
            self.get_logger().error(f"Erreur MAVLink : {e}")
            raise RuntimeError("Impossible de se connecter à MAVLink.")
        self.get_logger().info("En attente du heartbeat MAVLink...")
        self.mavlink_connection.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie !")
        
        ### IN MAV
        self.subscription_GPS_fire_coor = self.create_subscription(GeoPoint, 'GPS_fire_coor', self.lc_GPS_fire_coor, 10)

        self.get_logger().info("NODE MAV_manager STARTED.")

    def lc_GPS_fire_coor(self, GPS_fire_coor):
        # Log the received coordinates
        data = f"FIRE-Lat {GPS_fire_coor.latitude} Lon {GPS_fire_coor.longitude} Alt {GPS_fire_coor.altitude}"
        self.get_logger().info('SEND > %s' % data)
        
        # Send the text message using STATUSTEXT
        self.mavlink_connection.mav.statustext_send(
            severity=mavutil.mavlink.MAV_SEVERITY_EMERGENCY,  # Severity level
            text=data.encode('utf-8')             # Message text, encoded as UTF-8
        )

def main(args=None):
    rclpy.init(args=args)
    node = MAVManagerPopeye()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
