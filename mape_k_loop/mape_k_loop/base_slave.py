import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class BaseSlave(Node):
    def __init__(self, node_name ="slave"):
        super().__init__(node_name)
        self.state = 'idle'
        self.namespace = self.get_namespace().strip('/')
        self.create_subscription(String, f'/{self.namespace}/{node_name}/execution_command', self.command_callback)
        self.worker_thread = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker_thread.start()

    def command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")
        if command == 'start':
            if self.state in ['idle', 'paused']:
                self.state = 'running'
        elif command == 'pause':
            if self.state == 'running':
                self.state = 'paused'
        elif command == 'stop':
            self.state = 'stopped'
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def worker_loop(self):
        while rclpy.ok() and self.state != 'stopped':
            if self.state == 'running':
                self.do_task()
            elif self.state == 'paused':
                time.sleep(0.5)
            else:
                time.sleep(0.1)

    def do_task(self):
        raise NotImplementedError("Subclasses must implement do_task()")