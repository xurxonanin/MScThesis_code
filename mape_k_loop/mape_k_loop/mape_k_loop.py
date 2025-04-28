import rclpy
from rclpy.node import Node
from mape_k_loop.monitor import Monitor
from mape_k_loop.planning import Plan
from mape_k_loop.analyze import Analyze
from mape_k_loop.execute import Execute
import sys
class MAPE_K(Node):
    def __init__(self):
        super().__init__(f'mape_k_loop_node')

        # self.analyze = Analyze(name)
        # self.planning = Planning(name)
        # self.execute = Execute(name)
        self.get_logger().info(f'mape_k_loop_node loop started')


def main():
    print("Loop started")
    rclpy.init()
    mape_k = MAPE_K()
    rclpy.spin(mape_k)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()