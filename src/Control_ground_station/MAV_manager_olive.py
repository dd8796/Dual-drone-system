#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoint
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil
import sys

class MAVManagerOlive(Node):
    def __init__(self):
        super().__init__('MAV_manager_olive', namespace='CGS')

        # Connexion à MAVLink
        try:
            # ADRUPILOT SITL
            self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5773', baud=115200)
            # PX4 SITL
            # self.mavlink_connection = mavutil.mavlink_connection('udp:127.0.0.1:14031')
            # RADIO
            # self.mavlink_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)     
        except Exception as e:
            self.get_logger().error(f"Erreur MAVLink : {e}")
            raise RuntimeError("Impossible de se connecter à MAVLink.")
        self.get_logger().info("En attente du heartbeat MAVLink...")
        self.mavlink_connection.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie !")
                
        ### OUT MAV
        self.publisher_GPS_fire_coor = self.create_publisher(GeoPoint, 'GPS_fire_coor', 10)
        self.timer = self.create_timer(0.1, self.tc_GPS_fire_coor)

        # Launch return from Olive
        # for i in range(1, 255):  # Pour toutes les cibles possibles
        self.mavlink_connection.mav.request_data_stream_send(
            1,  # Target System ID
            mavutil.mavlink.MAV_COMP_ID_ALL,  # Target Component ID
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  # Type de données
            10,  # Fréquence de mise à jour (Hz)
            1  # Activer le flux (0 pour désactiver)
        )

        self.get_logger().info("NODE MAV_manager STARTED.")

    def tc_GPS_fire_coor(self):
        text_msg = self.mavlink_connection.recv_match(type="STATUSTEXT", blocking=False)
        if text_msg is None: # Obligatoire pour passer si c'est du bruit
            return
        self.get_logger().info('RECEIVED > %s' % text_msg.text)
        if 'FIRE' in text_msg.text:
            # Create a GeoPoint message
            data = text_msg.text.split()
            geopoint_msg = GeoPoint()
            geopoint_msg.latitude = float(data[1])
            geopoint_msg.longitude = float(data[3])
            geopoint_msg.altitude = float(data[5])

            # Publish the GeoPoint message
            self.publisher_GPS_fire_coor.publish(geopoint_msg)        

def main(args=None):
    rclpy.init(args=args)
    node = MAVManagerOlive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
