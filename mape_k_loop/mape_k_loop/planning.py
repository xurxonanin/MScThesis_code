import rclpy
from rclpy.node import Node
import redis
from sensor_msgs.msg import LaserScan
import json

class Plan(Node):
    def __init__(self):
        super().__init__('plan')
        
        self.get_logger().info('plan node started')

        self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
        if self.redis_client.ping():
            self.get_logger().info('Connected to Redis server')
            scan_value = self.redis_client.hgetall('latest_scan')
            if scan_value:
                ranges = json.loads(scan_value['ranges'])  # Convert JSON string back to a Python list
                intensities = json.loads(scan_value['intensities'])  # Convert JSON string back to a Python list
            self.get_logger().info(f'Value from Redis scan: ranges: {scan_value}')
            position_value = self.redis_client.hgetall('latest_pose')
            self.get_logger().info(f'Value from Redis position: {position_value}:{scan_value}')
        
        else:
            self.get_logger().error('Failed to connect to Redis server')

    # Add a timer callback to periodically read from database
    def read_from_db(self):
        current_scan = self.redis_client.hgetall('latest_scan')
        if current_scan:
            ranges = json.loads(current_scan['ranges'])
            intensities = json.loads(current_scan['intensities'])
        current_position = self.redis_client.hgetall('latest_pose')
        if current_position:
            position = json.loads(current_position['position'])
            orientation = json.loads(current_position['orientation'])

def main(args=None):
    rclpy.init(args=args)
    plan = Plan()
    rclpy.spin(plan)
    rclpy.shutdown()