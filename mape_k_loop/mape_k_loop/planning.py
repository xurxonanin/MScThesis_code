import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
import rclpy
from rclpy.node import Node
import redis
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
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
        try:
            self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)    
            if not self.redis_client.ping():
                self.get_logger().info('Connected to Redis server')
        except ConnectionError as e:
            self.get_logger().error(f'Failed to connect to Redis server: {e}')
            raise RuntimeError("Redis unavailable, aborting Planning node") from e
        self.get_logger().info(f'Companion namespace: {self.namespace_companion}')
        self.anomaly_client = self.create_client(CheckAnomaly,
            f'/{self.namespace_companion}/check_anomaly',
            callback_group=self._client_group
        )
        self.anomaly_topic  = self.create_subscription(
            String,
            f'{self.namespace}/anomaly',
            self.anomaly_callback,
            self.qos
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
        req = CheckAnomaly.Request()
        self.get_logger().info('Polling companion for anomaly state…')
        future = self.anomaly_client.call_async(req)

        # Block until we get an answer (or timeout)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            self.get_logger().warn('Planning: no response from companion, will retry next cycle')
            return False

        companion_anomaly = future.result().anomaly
        if companion_anomaly:
            self.get_logger().info('Planning: companion has an anomaly → recompute plan')
            # ← insert your replanning logic here
        else:
            self.get_logger().info('Planning: no anomaly, stick to current plan')
            # ← optional default plan
        return True
        
    

    def anomaly_callback(self, msg):
        self.get_logger().info(f'Anomaly detected: {msg.data}')
        # Check if the anomaly is present
        
        # If the anomaly is present, retrieve the state from the companion robot
        self.get_logger().info('Anomaly detected. Retrieving state from companion robot...')
        try:
            peer_anomaly_present = self.retrieve_peer_state()
        except Exception as e:
            self.get_logger().error(f'Failed to retrieve state: {e}')
            
        if peer_anomaly_present:
            self.get_logger().info('Anomaly detected. Change the plan.')
        
def main(args=None):
    rclpy.init(args=args)
    monitor = Planning()
    rclpy.spin(monitor)
    rclpy.shutdown()
