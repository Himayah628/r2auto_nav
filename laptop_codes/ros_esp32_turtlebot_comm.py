import rclpy
import paho.mqtt.client as mqtt
from rclpy.node import Node

from std_msgs.msg import String


#class MinimalPublisher(Node):

#    def __init__(self):
#        super().__init__('minimal_publisher')
#        self.publisher_ = self.create_publisher(String, 'topic', 10)
#        timer_period = 0.5  # seconds
#        self.timer = self.create_timer(timer_period, self.timer_callback)
#        self.i = 0

#    def timer_callback(self):
#        msg = String()
#        msg.data = 'Hello World: %d' % self.i
#        self.publisher_.publish(msg)
#        self.get_logger().info('Publishing: "%s"' % msg.data)
#        self.i += 1

# MQTT variables
MQTT_ADDRESS = 'localhost'
MQTT_TOPIC = 'table'

class Communication(Node):

    def __init__(self):
        super().__init__('laptop')

	# subscriber to table topic
        self.subscription = self.create_subscription(
            String,
            'table',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # subscriber to esp32_status topic
        self.subscription1 = self.create_subscription(
            String,
            'esp32_status',
            self.listener1_callback,
            10)
        self.subscription1

	# publisher for table topic for debugging
        self.publisher_ = self.create_publisher(String, 'table', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

	# attempt to publish multiple topics
        #self.publisher1_ = self.create_publisher(String, 'topic1', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback1)
        #self.i = 0

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect(MQTT_ADDRESS, 1883)
        self.mqtt_client.loop_start()

    # listener callback function for table
    def listener_callback(self, msg):
        self.get_logger().info('Table no.: "%s"' % msg.data)

    def listener1_callback(self, msg):
        self.get_logger().info('Received from Turtlebot: "%s"' % msg.data)
        self.mqtt_client.publish("esp32/status", msg.data)
    # publisher timer callback function
    #def timer_callback(self):
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1

    #def timer_callback1(self):
        #msg = String()
        #msg.data = 'Topic1: %d' % self.i
        #self.publisher1_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1

    def on_connect(self, client, userdata, flags, rc):
        print('Connected with result code ' + str(rc))
        client.subscribe(MQTT_TOPIC)

    def on_message(self, client, userdata, msg):
        print(msg.topic + ' ' + str(msg.payload.decode("utf-8")))
        ros_msg = String()
        ros_msg.data = msg.payload.decode("utf-8")
        self.publisher_.publish(ros_msg)
        self.get_logger().info('Recieved from ESP32: "%s"' % ros_msg.data)

def main(args=None):
    rclpy.init(args=args)

#    minimal_publisher = MinimalPublisher()
    laptop_comm = Communication()

#    rclpy.spin(minimal_publisher)
    rclpy.spin(laptop_comm)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laptop_comm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
