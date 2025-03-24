import rclpy
from rclpy.node import Node
import rtamt



class Analyze(Node):
    def __init__(self, name):
        super().__init__(f'{name}_monitor')
        print("Analyze node started")