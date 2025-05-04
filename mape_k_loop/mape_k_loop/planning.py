import rclpy
from rclpy.node import Node
import redis
from sensor_msgs.msg import LaserScan
import json
import signal

class Plan(Node):
    def __init__(self):
        super().__init__('plan')
        self.namespace = self.get_namespace().strip('/')
        self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
        if self.redis_client.ping():
            self.get_logger().info('Connected to Redis server')
        else:
            # If the database connection fails, set up a signal handler to stop execution
            self.get_logger().error('Failed to connect to Redis server')
            def stop_execution(signal, frame):
                self.get_logger().info('Stopping execution...') 
                rclpy.shutdown()
                exit(0)

            signal.signal(signal.SIGINT, stop_execution)
        self.reader = self.create_timer(0.5, self.read_from_db)
        self.get_logger().info('Plan node started')


        # Add timer to periodically check for the anomaly topic.

        # Strategy decided: RTAMT monitor to check robustness of the system
        # Occlusion detection:
        # 1. Check if the occlusion is present
        # 2. If occlusion is present what option is better
        # Use a RTAMT monitor to check if the latency is small enough and the battery is enough
        # This will be the decision maker.