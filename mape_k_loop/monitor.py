import rclpy
from rclpy.node import Node
import rtamt
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Monitor(Node):
    def __init__(self):
        super.__init__('monitor')
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

    def laser_callback(self, msg):
        print(msg.ranges)