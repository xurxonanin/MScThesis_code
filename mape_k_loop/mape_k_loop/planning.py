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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from mape_k_loop.base_slave import BaseSlave

class Planning(BaseSlave):
    def __init__(self):
        super().__init__('planning')
        self.namespace = self.get_namespace().strip('/')
        self._client_group = MutuallyExclusiveCallbackGroup()
        match = re.match(r'(tb)(\d+)',self.namespace)
        if match:
            letters, numbers = match.groups()
            numbers = (int(numbers) % 2) + 1  # Parse numbers to integer before performing modulo operation
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
        self.get_logger().info(f'Companion namespace: {self.namespace_companion}')
        self.anomaly_client = self.create_client(CheckAnomaly,
            f'/{self.namespace_companion}/check_anomaly',
            callback_group=self._client_group
        )
        self.anomaly_topic  = self.create_subscription(
            String,
            f'{self.namespace}/anomaly',
            self.anomaly_callback,
            1
        )
        # read_from_db yet to be implemented
        self.get_logger().info('Plan node started')


        # Add timer to periodically check for the anomaly topic.

        # Strategy decided: RTAMT monitor to check robustness of the system
        # Occlusion detection:
        # 1. Check if the occlusion is present
        # 2. If occlusion is present what option is better
        # Use a RTAMT monitor to check if the latency is small enough and the battery is enough
        # This will be the decision maker.
    def do_task(self):
        request = CheckAnomaly.Request()
        self.get_logger().info('Calling companion to check its state...')
        future = self.anomaly_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.done() and not future.cancelled():
            self.get_logger().info('Companion state retrieved successfully.')
        else:
            self.get_logger().error('Failed to retrieve companion state.')
            return False
        if future.result() is not None:
            return future.result().anomaly
        else:
            raise RuntimeError(
                'Service call failed: %r' % (future.exception(),)
            )
    

    def anomaly_callback(self, msg):
        self.get_logger().info(f'Anomaly detected: {msg.data}')
        # Check if the anomaly is present
        
        # If the anomaly is present, retrieve the state from the companion robot
        self.get_logger().info('Anomaly detected. Retrieving state from companion robot...')
        try:
            peer_anomaly_present = self.retrieve_peer_state()
        except Exception as e:
            self.get_logger().error(f'Failed to retrieve state: {e}')
            return
        if peer_anomaly_present:
            self.get_logger().info('Anomaly detected. Change the plan.')
        
def main(args=None):
    rclpy.init(args=args)
    planning = Planning()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(planning)
    try:
        executor.spin()
    finally:
        rclpy.spin(planning)
        rclpy.shutdown()
