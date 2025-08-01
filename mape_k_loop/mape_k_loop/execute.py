# execute.py
import os, sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rclpy
from mape_k_loop.base_slave import BaseSlave
from std_msgs.msg import String, Bool
from rclpy.qos import QoSPresetProfiles, QoSDurabilityPolicy

class Execute(BaseSlave):
    def __init__(self):
        # must pass exactly "execute" so the master’s topics line up
        super().__init__('execute')
        self.get_logger().info('Execute node started')

    def do_task(self) -> bool:
        # This will get called every self.period seconds.
        # Return False to retry, or True to signal “I’m done.”
        self.get_logger().info('🚀 Execute: performing action...')
        # ← insert your actual execution logic here
        return True

def main(args=None):
    rclpy.init(args=args)
    node = Execute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()