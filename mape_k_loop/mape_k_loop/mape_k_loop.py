import rclpy
from rclpy.node import Node
from mape_k_loop.monitor import Monitor
from mape_k_loop.planning import Planning
from mape_k_loop.analyze import Analyze
from mape_k_loop.execute import Execute
import sys
class MAPE_K(Node):
    def __init__(self,name):
        super().__init__(f'{name}_loop_node')
        self.monitor = Monitor(name)
        self.analyze = Analyze(name)
        self.planning = Planning(name)
        self.execute = Execute(name)
        self.get_logger().info(f'{name} loop started')


def main():
    print("Loop started")
    rclpy.init()
    node_name = None
    for arg in sys.argv:
        if arg.startswith("__node:="):
            node_name = arg.split(":=")[1]
            break

    if node_name is None:
        raise ValueError("Node name not provided in ROS 2 arguments.")
    mape_k = MAPE_K(node_name)
    rclpy.spin(mape_k)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()