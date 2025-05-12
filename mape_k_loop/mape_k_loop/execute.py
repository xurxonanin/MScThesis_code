import rclpy
from rclpy.node import Node
import rtamt



class Execute(Node):
    def __init__(self, name):
        super().__init__(f'execute')
    