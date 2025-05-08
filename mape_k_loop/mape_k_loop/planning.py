import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
import rclpy
from rclpy.node import Node
import redis
from sensor_msgs.msg import LaserScan
import json
import signal
from std_msgs.msg import String
import re
from mape_k_interfaces.srv import CheckAnomaly


class Plan(Node):
    def __init__(self):
        super().__init__('plan')
        self.namespace = self.get_namespace().strip('/')
        match = re.match(r'(tb)(\d+)',self.namespace)
        if match:
            letters, numbers = match.groups()
            numbers = (numbers%2)+1 # If tb1 => 1%2 = 1 => 1+1 = 2 || if tb2 => 2%2 = 0 => 0+1 = 1
            self.namespace_companion = f'{letters}{numbers}'
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
        self.anomaly_client = self.create_client(CheckAnomaly, f'{self.namespace_companion}/check_anomaly')
        self.anomaly_topic  = self.create_subscription(
            String,
            f'{self.namespace}/anomaly',
            self.anomaly_callback,
            2
        )
        
        self.get_logger().info('Plan node started')


        # Add timer to periodically check for the anomaly topic.

        # Strategy decided: RTAMT monitor to check robustness of the system
        # Occlusion detection:
        # 1. Check if the occlusion is present
        # 2. If occlusion is present what option is better
        # Use a RTAMT monitor to check if the latency is small enough and the battery is enough
        # This will be the decision maker.
        def retrieve_state(self):
            request = CheckAnomaly.Request()
            future = self.anomaly_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                return future.result().anomaly
            else:
                raise RuntimeError(
                    'Service call failed: %r' % (future.exception(),)
                )
        

        def anomaly_callback(self, msg):
            self.get_logger().info(f'Anomaly detected: {msg.data}')
            # Check if the anomaly is present
            if msg.data == 'True':
                # If the anomaly is present, check the battery and latency
                try:
                    anomaly_present = self.retrieve_state()
                except Exception as e:
                    self.get_logger().error(f'Failed to retrieve state: {e}')
                    return
                if anomaly_present:
                    self.get_logger().info('Anomaly detected. Change the plan.')
            else:
                # If the anomaly is not present, continue with normal operation
                self.get_logger().info('No anomaly detected. Continuing normal operation.')
        
        