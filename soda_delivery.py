import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np
import math
import cmath
import RPi.GPIO as GPIO
import time
# constants
rotatechange = 0.2
speedchange = 0.05
front_angle = 0
clockwise_angle = -90
anticlockwise_angle = 90
GPIOpin = 15
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIOpin,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
state = 0
table = None
table6max = 1.58
Left_IR = 16 #GPIO 16 -> Left IR out
Right_IR = 26 #GPIO 26 -> Right IR out
#setup
GPIO.setup(Left_IR,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(Right_IR,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)#setup


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
        self.table = None

        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        self.get_table = self.create_subscription(
            String,
            'table',
            self.set_table_no,
            10)
        self.get_table # prevent unused variable warning

        # create publisher to tell esp32 to poll for user input
        self.esp32_start = self.create_publisher(String, 'esp32_status', 10)
    
    def irsensor(self):
         while 1:
             if(GPIO.input(Left_IR)==True and GPIO.input(Right_IR)==True):  #stop
                 self.get_logger().info('In stopbot,both detected')
                 # publish to cmd_vel to move TurtleBot
                 twist = Twist()
                 twist.linear.x = 0.0
                 twist.angular.z = 0.0
                 rclpy.spin_once(self)
                 # time.sleep(1)
                 self.publisher_.publish(twist)
                 break
             elif(GPIO.input(Left_IR)==False and GPIO.input(Right_IR)==True):  
                 self.get_logger().info('left one detects')
                 twist = Twist()
                 twist.linear.x = -0.05
                 twist.angular.z = 0.3
                 self.publisher_.publish(twist)
                 rclpy.spin_once(self)
 
             elif(GPIO.input(Left_IR)==True and GPIO.input(Right_IR)==False): 
                 self.get_logger().info('right one detects')
                 twist = Twist()
                 twist.linear.x = -0.05
                 twist.angular.z = -0.3
                 self.publisher_.publish(twist)
                 rclpy.spin_once(self)
 
             else:  #forward
                 self.get_logger().info('none')
                 twist = Twist()
                 twist.linear.x =-0.05
                 twist.angular.z = 0.0
                 self.publisher_.publish(twist)
                 rclpy.spin_once(self)
                 
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
        rclpy.spin_once(self)

    def moveforward(self, detect_angles, detect_distanct, reverse=False, angle=10):
        front_angles = range(detect_angles-angle,detect_angles+angle+1,1)
        twist = Twist()
        twist.linear.x = 0.1
        if(reverse):
            twist.linear.x = -0.1
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        rclpy.spin_once(self)
        time.sleep(5)
        while rclpy.ok():
            if self.laser_range.size != 0:
                # check distances in front of TurtleBot and find values less
                # than detect_distanct
                lri = (self.laser_range[front_angles]<float(detect_distanct)).nonzero()
                self.get_logger().info('Distances: %s' % str(lri))

                # if the list is not empty
                if(len(lri[0])>0):
                    self.get_logger().info('Stopping')
                    # publish to cmd_vel to move TurtleBot
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    # time.sleep(1)
                    self.publisher_.publish(twist)
                    break
            rclpy.spin_once(self)
            
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
                
    
    def travelling_to_table(self):
        while True:
            rclpy.spin_once(self)
            if self.table != None:
                break

        while True:
            if not self.proximity():
                self.get_logger().info('Metal detected')
                break

        if(self.table == 5):
            self.orientation_check(175)
            self.moveforward(0, 0.50)
            self.rotatebot(anticlockwise_angle)
            self.orientation_check(175) #use lidar to confirm
            self.moveforward(0, 0.35) #reach table 5
        else:
            self.rotatebot(anticlockwise_angle)
            if (self.table == 1 or self.table == 2):
                self.moveforward(0, 0.35)
            else:
                self.moveforward(0,0.45)
            if(self.table != 1):
                self.rotatebot(clockwise_angle)
                self.orientation_check(175)
                if(self.table == 6):
                    self.moveforward(0, 0.5)
                    self.rotatebot(anticlockwise_angle)
                    self.orientation_check(265)# use lidar to confirm
                    #stop after passing line
                    self.moveforward(0, 1.5)
                    # move forward while checking the distance at 90. if distance at 90 is smaller than maxdistance to table then stop and rotate anticlockwise 90. moveforward(0, 0.3)
                    self.moveforward(90,table6max,angle=0)
                    self.rotatebot(anticlockwise_angle)
                    self.moveforward(0,0.35) #reach table 6
                    
                else:
                    if(self.table == 2):
                        self.moveforward(100, 0.45, angle=1)
                    else:
                        self.moveforward(100, 0.5, angle=1) #keep it at 5
                    if(self.table == 2):
                        self.rotatebot(anticlockwise_angle) #reach table 2
                    elif(self.table == 3):
                        self.rotatebot(clockwise_angle) #reach table 3
                    elif(self.table == 4):
                        self.moveforward(0, 0.5)  #reach table 4
                        self.rotatebot(clockwise_angle)
                
        self.get_logger().info('Successfully delivered to table %d' % self.table)
        
    def returning_from_table(self):
        while True:
            if self.proximity():
                self.get_logger().info('Metal Not detected')
                break

        if (self.table == 6):
            self.moveforward(180, 0.5, reverse = True)
            self.rotatebot(clockwise_angle)
            self.orientation_check(262)
            self.moveforward(180, 0.40, reverse = True)
            self.rotatebot(clockwise_angle)
        if (self.table == 4):
            self.rotatebot(anticlockwise_angle)
        if(self.table == 4 or self.table == 6):
            self.moveforward(180, 0.70, reverse=True)
            self.rotatebot(anticlockwise_angle)
            self.irsensor()
        elif(self.table == 3):
            self.rotatebot(anticlockwise_angle)
            self.moveforward(180, 0.40, reverse=True)
            self.rotatebot(anticlockwise_angle)
            self.irsensor()
        elif(self.table == 2):
            self.rotatebot(clockwise_angle)
            self.moveforward(90, 0.40, reverse=True)
            self.rotatebot(anticlockwise_angle)
            self.irsensor()
        elif(self.table == 5):
            self.moveforward(180, 0.60, reverse = True)
            self.rotatebot(clockwise_angle)
            self.irsensor()
        else:
            self.irsensor()

        self.get_logger().info("Successfully returned from table %d" % self.table)
        self.table = None
        self.start_esp32()

    def proximity(self):
        state = GPIO.input(GPIOpin)
        return state
    
    def set_table_no(self, msg):
        self.get_logger().info('Received from ESP32: "%s"' % msg.data)
        self.table = int(msg.data)

    def start_esp32(self):
        msg = String()
        msg.data = 'start'
        self.esp32_start.publish(msg)
         
def main(args=None):
    rclpy.init(args=args)
    auto_nav = AutoNav()
    while(True):
        rclpy.spin_once(auto_nav)
        auto_nav.travelling_to_table()
        rclpy.spin_once(auto_nav)
        auto_nav.returning_from_table()
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

if __name__ == '__main__':
    main()
