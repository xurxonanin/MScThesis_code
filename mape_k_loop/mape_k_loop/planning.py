import rclpy
from rclpy.node import Node
import redis
from sensor_msgs.msg import LaserScan
import json
import signal

class Plan(Node):
    def __init__(self):
        super().__init__('plan')
        
    