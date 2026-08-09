#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint

class AnalyseCAM(Node):
    def __init__(self):
        super().__init__('analyse_CAM', namespace='OLIVE')
        
        self.publisher_GPS_coor_fire = self.create_publisher(GeoPoint, 'OUT/GPS_fire_coor', 10)
        self.timer = self.create_timer(1, self.tc_GPS_fire_coor)  # 1 second interval

        self.subscription_GPS_olive_coor = self.create_subscription(GeoPoint, 'IN/GPS_olive_coor', self.lc_GPS_olive_coor, 10)

        # Pour les tests
        self.latitude = 0.
        self.longitude = 0.
        self.altitude = 0.

        self.get_logger().info("NODE analyse_cam STARTED.")
        
    def tc_GPS_fire_coor(self):
        # Création du message GPSData
        gps_message = GeoPoint()
        gps_message.latitude = self.latitude
        gps_message.longitude = self.longitude
        gps_message.altitude = self.altitude

        # Debug
        self.latitude += 0.001
        self.longitude += 0.001
        self.altitude += 0.001

        # Publication
        self.publisher_GPS_coor_fire.publish(gps_message)
        # self.get_logger().info(f"Published: Latitude={gps_message.latitude}, Longitude={gps_message.longitude}, Altitude={gps_message.altitude}")

    def lc_GPS_olive_coor(self, GPS_olive_coor):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AnalyseCAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
