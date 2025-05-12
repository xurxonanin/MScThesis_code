import rclpy
from rclpy.node import Node
from mape_k_loop.monitor import Monitor
from mape_k_loop.planning import Planning
from mape_k_loop.analyze import Analyze
from mape_k_loop.execute import Execute
import sys
import rtamt
from std_msgs.msg import String, Bool

# Master Node for MAPE-K loop
class MAPE_K(Node):
    def __init__(self):
        super().__init__(f'mape_k_loop_master_node')
        self.namespace = self.get_namespace().strip('/')

        # Sends commands to the nodes indicating the start, pause, resume, or stop of the execution
        self.pub_monitor = self.create_publisher(String, f'/{self.namespace}/monitor/execution_command', 1)
        self.pub_analysis = self.create_publisher(String, f'/{self.namespace}/analyze/execution_command', 1)
        self.pub_planning = self.create_publisher(String, f'/{self.namespace}/planning/execution_command', 1)
        self.pub_execution = self.create_publisher(String, f'/{self.namespace}/execute/execution_command', 1)

        # List of commands to be sent
        self.commands = ['start','pause','resume','stop']
        # Current stages
        self.stages = ['monitor','analyze','planing','execute']

        self.current_state_index = 0
        self.current_state = self.stages[self.current_state_index]

        self.monitor_state = False
        self.analyze_state = True
        self.planning_state = True
        self.execute_state = True


        self.monitor_listener = self.create_subscription(
            Bool,
            '/{self.namespace}/monitor_finished',
            self.status_callback,
            1,
            status = self.monitor_state
        )

        self.analysis_listener = self.create_subscription(
            Bool,
            '/{self.namespace}/analysis_finished',
            self.status_callback,
            1,
            status = self.analyze_state
        )

        self.planning_listener = self.create_subscription(
            Bool,
            '/{self.namespace}/planning_finished',
            self.status_callback,
            1,
            status = self.planning_state
        )

        self.execute_listener = self.create_subscription(
            Bool,
            '/{self.namespace}/execute_finished',
            self.status_callback,
            1,
            status = self.execute_state
        )
        self.create_timer(0.5, self.switch_state)

        self.get_logger().info(f'mape_k_loop_node loop started')
    

    
    def switch_state(self):
        """
        Switches the state of the MAPE-K loop based on the current state and the status of the nodes.
        """
        match self.current_state:
            case 'monitor':
                if self.monitor_state:
                    self.pub_monitor.publish(String(data='pause'))
                    self.get_logger().info(f'monitor node finished')
                    self.monitor_state = False
                    self.current_state_index += 1 % len(self.stages)
                    self.current_state = self.stages[self.current_state_index]
                    msg = String()
                    msg.data = 'start'
                    self.pub_analysis.publish(msg)
                else:
                    return
            case 'analyze':
                if self.analyze_state:
                    self.pub_analysis.publish(String(data='pause'))
                    self.get_logger().info(f'analyze node finished')
                    self.analyze_state = False
                    self.current_state_index += 1 % len(self.stages)
                    self.current_state = self.stages[self.current_state_index]
                    msg = String()
                    msg.data = 'start'
                    self.pub_planning.publish(msg)
                else:
                    return
            case 'planing':
                if self.planning_state:
                    self.pub_planning.publish(String(data='pause'))
                    self.get_logger().info(f'planning node finished')
                    self.planning_state = False
                    self.current_state_index += 1 % len(self.stages)
                    self.current_state = self.stages[self.current_state_index]
                    msg = String()
                    msg.data = 'start'
                    self.pub_execution.publish(msg)
                else:
                    return
            case 'execute':
                if self.execute_state:
                    self.pub_execution.publish(String(data='pause'))
                    self.get_logger().info(f'execute node finished')
                    self.execute_state = False
                    self.current_state_index += 1 % len(self.stages)
                    self.current_state = self.stages[self.current_state_index]
                    msg = String()
                    msg.data = 'start'
                    self.pub_monitor.publish(msg)
            case 'stop':
                self.get_logger().info(f'mape_k_loop_node loop stopped')
                rclpy.shutdown()
                sys.exit(0)
            case _:
                return
    def status_callback(self, msg, status):
        if msg.data != status:
            status = msg.data
        else:
            return
def main():
    print("Loop started")
    rclpy.init()
    mape_k = MAPE_K()
    rclpy.spin(mape_k)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()