import rclpy
from rclpy.node import Node
import rtamt
from sensor_msgs.msg import LaserScan


class Monitor(Node):
    def __init__(self, name):
        super().__init__(f'{name}_monitor')
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

    def laser_callback(self, msg):
        print(msg.ranges)