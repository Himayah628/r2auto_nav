import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import RPi.GPIO as GPIO
import time

class Adjustment(Node):

    def __init__(self):
        super().__init__('lidar_test')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.scan_subscription
        self.laser_range = np.array([])

    def scan_callback(self, msg):
        self.laser_range = np.array(msg.ranges)
        self.laser_range[self.laser_range==0] = np.nan

    # require positive input for arrray splicing, does not work with angles 355-359, 0-5
    def orientation_check(self, angle):
        twist = Twist()
        while rclpy.ok():
            if self.laser_range.size != 0:
                distance_at_angles = self.laser_range[angle-5:angle+6]
                if (np.sum(np.isnan(distance_at_angles)) == 0):
                    self.get_logger().info("Array Info: %s" % np.array2string(distance_at_angles))
                    shortest_angle = np.nanargmin(distance_at_angles)
                    if shortest_angle < 5:
                        twist.linear.x = 0.0
                        twist.angular.z = -0.05
                    elif shortest_angle > 5:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.05
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.publisher_.publish(twist)
                        rclpy.spin_once(self)
                        break
                else:
                    twist.angular.z = 0.0
                #twist.angular.z = 0.1
                self.publisher_.publish(twist)
                rclpy.spin_once(self)
        self.get_logger().info("Adjustment Complete")

def main(args=None):
    rclpy.init(args=args)
    adjust = Adjustment()
    angle = int(input("Enter angle: "))
    rclpy.spin_once(adjust)
    adjust.orientation_check(angle)
    #rclpy.spin_once(auto_nav)
    #auto_nav.irsensor()
    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    adjust.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
