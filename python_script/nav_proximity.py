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
# constants
rotatechange = 0.1
speedchange = 0.05
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
Table = int(input("table number: "))
clockwise_angle = 90
anticlockwise_angle = -90
GPIOpin = 15
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIOpin,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
state=0
#Left_IR = GPIO(16, direction = GPIO.IN) #GPIO 16 -> Left IR out
#Right_IR = GPIO(18,direction = GPIO.IN) #GPIO 18 -> Right IR out
#setup

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
    def detectMetal():
        state = GPIO.input(GPIOpin)
        if not state:
            self.get_logger().info("Metal Detected")
        else :
            self.get_logger().info("Metal Not Detected")
            
            
    def irsensor(self):
         while 1:
             if(GPIO.read(16)==False and GPIO.read(18)==False): #rotate 90 clockwise    
                 self.rotate(90)
                 break
             else:  #forward
                 MotorDriver.set_speed(speed1 = 0, speed2 = 0)
         while 1:
             if(GPIO.read(16)==False and GPIO.read(18)==False):  #stop
                 self.get_logger().info('In stopbot')
                 # publish to cmd_vel to move TurtleBot
                 twist = Twist()
                 twist.linear.x = 0.0
                 twist.angular.z = 0.0
                 # time.sleep(1)
                 self.publisher_.publish(twist)
             elif(GPIO.read(16)==False and GPIO.read(18)==True): #turn left 
                 twist = Twist()
                 twist.linear.x = 0.1
                 twist.angular.z = 0.1
                 self.publisher_.publish(twist)
 
             elif(GPIO.read(2)==True and GPIO.read(3)==False): #turn right
                 twist = Twist()
                 twist.linear.x = 0.1
                 twist.angular.z = -0.1
                 self.publisher_.publish(twist)
 
             else:  #forward
                 twist = Twist()
                 twist.linear.x = 0.1
                 twist.angular.z = 0.0
                 self.publisher_.publish(twist)

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def stopbot(self):
        
        while rclpy.ok():
            # self.get_logger().info(self.laser_range.size)
            if self.laser_range.size != 0:
                # check distances in front of TurtleBot and find values less
                # than stop_distance
                lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                self.get_logger().info('Distances: %s' % str(lri))

                # if the list is not empty
                if(len(lri[0])>0):
                    self.get_logger().info('Stopbot')
                    # publish to cmd_vel to move TurtleBot
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    # time.sleep(1)
                    self.publisher_.publish(twist)
            rclpy.spin_once(self)
    def travelling_to_table1(self):
        self.get_logger().info('inside the def')
        while True:
            detectMetal()
            if not state:
                break
        self.rotatebot(anticlockwise_angle)
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.stopbot()
        
    def returning_from_table1(self):
        while True:
            if not detectMetal():
                break
        self.rotatebot(clockwise_angle)
        twist = Twist()
        twist.linear.x = -0.5
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.stopbot()


def main(args=None):
    rclpy.init(args=args)
    self.get_logger().info('inside the def') 
    auto_nav = AutoNav()
    rclpy.spin_once(auto_nav)
    auto_nav.travelling_to_table1()
    #rclpy.spin_once(auto_nav)
    #auto_nav.returning_from_table1()
    #rclpy.spin_once(auto_nav)
    #auto_nav.irsensor()
    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()



if __name__ == ' __main__':
    main()
