import rclpy
from rclpy.node import Node
from mape_k_loop.monitor import Monitor
from mape_k_loop.planning import Planning
from mape_k_loop.analyze import Analyze
from mape_k_loop.execute import Execute
from std_msgs.msg import String, Bool
from rclpy.qos import QoSPresetProfiles, QoSDurabilityPolicy
from functools import partial # For Python 3.8 and above, you can use functools.partial to pass the status argument

# Master Node for MAPE-K loop
class MAPE_K(Node):
    def __init__(self):
        super().__init__(f'mape_k_loop_master_node')
        self.namespace = self.get_namespace().strip('/')
        self.qos = QoSPresetProfiles.SYSTEM_DEFAULT.value
        self.qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        # Sends commands to the nodes indicating the start, pause, resume, or stop of the execution
        self.pub_monitor = self.create_publisher(String, f'/{self.namespace}/monitor/execution_command', self.qos)
        self.pub_analyze = self.create_publisher(String, f'/{self.namespace}/analyze/execution_command', self.qos)
        self.pub_planning = self.create_publisher(String, f'/{self.namespace}/planning/execution_command', self.qos)
        self.pub_execute = self.create_publisher(String, f'/{self.namespace}/execute/execution_command', self.qos)

        # List of commands to be sent
        self.commands = ('start','pause')
        # Current stages
        self.stages = ('monitor','analyze','planning','execute')

        self.current_state = 'monitor'

        self.monitor_state = False
        self.analyze_state = False
        self.planning_state = False
        self.execute_state = False

        

        self._monitor_sub = self.create_subscription(
            Bool,
            f'/{self.namespace}/monitor_finished',
            partial(self.status_callback, stage='monitor'),
            self.qos
        )

        self._analyze_sub = self.create_subscription(
            Bool,
            f'/{self.namespace}/analyze_finished',
            partial(self.status_callback, stage='analyze'),
            self.qos
        )

        self._planning_sub = self.create_subscription(
            Bool,
            f'/{self.namespace}/planning_finished',
            partial(self.status_callback, stage='planning'),
            self.qos
        )

        self._execute_sub = self.create_subscription(
            Bool,
            f'/{self.namespace}/execute_finished',
            partial(self.status_callback, stage='execute'),
            self.qos
        )
        self.create_timer(0.5, self.switch_state)

        self.get_logger().info(f'mape_k_loop_node loop started')
        self.get_logger().info(f'{self.namespace}: Starting Monitor stage')
        self.pub_monitor.publish(String(data='start'))

    
    def switch_state(self):
        """
        Switches the state of the MAPE-K loop based on the current state and the status of the nodes.
        """
        match self.current_state:
            case 'monitor':
                if not self.monitor_state:
                    return    
                # state is monitor, monitor has finished and analyze did not start
                self.pub_monitor.publish(String(data='pause'))
                self.get_logger().info('Monitor stage finished; launching Analyze')
                self.monitor_state = False
                self.current_state = 'analyze'
                self.pub_analyze.publish(String(data='start'))
            
            case 'analyze':
                if not self.analyze_state:
                    return
                # state is analyze, analyze is finished and planning did not start
                self.pub_analyze.publish(String(data='pause'))
                self.get_logger().info('Analyze stage finished; launching Planning')
                self.analyze_state = False
                self.current_state = 'planning'
                self.pub_planning.publish(String(data='start'))
            
            case 'planning':
                if not self.planning_state:
                    return
                # state is analyze, analyze is finished and planning did not start
                self.pub_planning.publish(String(data='pause'))
                self.get_logger().info('Planning stage finished; launching Execute')
                self.planning_state = False
                self.current_state = 'execute'
                self.pub_execute.publish(String(data='start'))

            case 'execute':
                if not self.execute_state:
                    return
                # state is analyze, analyze is finished and planning did not start
                self.pub_execute.publish(String(data='pause'))
                self.get_logger().info('Execute stage finished; launching Monitor')
                self.execute_state = False
                self.current_state = 'monitor'
                self.pub_monitor.publish(String(data='start'))
                
            case _:
                self.get_logger().warn(f'Unknown state {self.current_state}')
                    



    def status_callback(self, msg, stage):
        if msg.data:
            setattr(self, f"{stage}_state", True)
                
def main():
    rclpy.init()
    mape_k = MAPE_K()
    rclpy.spin(mape_k)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()