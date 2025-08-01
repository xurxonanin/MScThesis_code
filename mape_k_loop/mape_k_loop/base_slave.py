import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading
import time
from rclpy.qos import QoSPresetProfiles, QoSDurabilityPolicy

class BaseSlave(Node):
    def __init__(self, node_name ="slave", period = 1.0):
        super().__init__(node_name)
        self.state = 'idle'
        self.qos = QoSPresetProfiles.SYSTEM_DEFAULT.value
        self.qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.namespace = self.get_namespace().strip('/')
        self.create_subscription(String, f'/{self.namespace}/{node_name}/execution_command', self.command_callback, self.qos)
        self.finished_publisher = self.create_publisher(
            Bool,
            f'/{self.namespace}/{node_name}_finished',
            self.qos
        )
        self.period = period
        self._timer = None
        

    def command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")
        if command == 'start':
            if self._timer is None:
                self.finished_publisher.publish(Bool(data=False))
                self._timer = self.create_timer(
                    self.period,
                    self.worker_loop
                )
                self.state = 'running'
        elif command == 'pause':
            if self._timer is not None:
                self._timer.cancel()
                self.destroy_timer(self._timer)
                self._timer = None
                self.state = 'paused'
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def worker_loop(self):
        try:
            finished = self.do_task()
        except Exception as e:
            self.get_logger().error(f"Error in do_task: {e}")
            finished = True
        
        if finished:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
            self.finished_publisher.publish(Bool(data=True))
        

    def do_task(self):
        raise NotImplementedError("Subclasses must implement do_task()")