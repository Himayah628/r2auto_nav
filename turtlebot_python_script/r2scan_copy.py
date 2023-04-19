import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np


class Scanner(Node):
    def __init__(self):
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription #prevent unused variable warning

    def listener_callback(self, msg):
        laser_range = np.array(msg.ranges)
        laser_range[laser_range==0] = np.nan
        lr2i = np.nanargmin(laser_range)
        self.get_logger().info('Shortest distance at %i degrees' % lr2i)

def main(args=None):
    rclpy.init(args=args)
    scanner = Scanner()
    rclpy.spin(scanner)
    scanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
