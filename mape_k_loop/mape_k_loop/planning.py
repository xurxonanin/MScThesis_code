import rclpy
from rclpy.node import Node
import rtamt
from sensor_msgs.msg import LaserScan


class Planning(Node):
    def __init__(self, name):
        super().__init__(f'{name}_planning')
        print("Planning node started")