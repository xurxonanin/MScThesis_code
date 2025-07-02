import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
import rclpy
from rclpy.node import Node
import redis
from sensor_msgs.msg import LaserScan
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import json
import math
from threading import Lock
import signal
import sys
from mape_k_loop.base_slave import BaseSlave
from rclpy.qos import QoSPresetProfiles
class Monitor(BaseSlave):
    def __init__(self):
        super().__init__(f'monitor')
        self.laser_sub = self.create_subscription(LaserScan, 
            'scan',
            self.laser_callback, 
            QoSPresetProfiles.SYSTEM_DEFAULT.value
        )
        
        self.namespace = self.get_namespace().strip('/')
        self.create_subscription(
            String,
            f'/{self.namespace}/monitor/execution_command',
            self.command_callback,
            QoSPresetProfiles.SYSTEM_DEFAULT.value
        )
        self.odom_sub = self.create_subscription(Odometry,
            'odom',
            self.odom_callback, 
            QoSPresetProfiles.SYSTEM_DEFAULT.value
        )
        self.get_logger().info(f'/monitor node started')
        self.finished_publisher = self.create_publisher(
            Bool,
            f'/{self.namespace}/monitor_finished',
            1
        )
        self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
        self.current_scan = None
        self.current_position = None
        if self.redis_client.ping():
            self.get_logger().info('Connected to Redis server')
        else:
            self.get_logger().error('Failed to connect to Redis server')
            # If the database connection fails, set up a signal handler to stop execution
            self.get_logger().error('Failed to connect to Redis server')
            def stop_execution(signal, frame):
                self.get_logger().info('Stopping execution...')
                rclpy.shutdown()
                exit(0)

            signal.signal(signal.SIGINT, stop_execution)
        self.lock = Lock()
        self.get_logger().info('Monitor node started')

    def euler_from_quaternion(self, quaternion):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        """
        r = R.from_quat(quaternion)
        # Convert to radians
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        return roll, pitch, yaw

    def laser_callback(self, msg):
        with self.lock:
            # Store the latest scan in attribute

            filtered_ranges = [
                x if not math.isinf(x) else msg.range_max
                for x in msg.ranges
            ]
            ranges_json = json.dumps(filtered_ranges)
            intensities_json = json.dumps(list(msg.intensities))
            self.current_scan = {
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'time_increment': msg.time_increment,
                'scan_time': msg.scan_time,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'ranges': ranges_json,
                'intensities': intensities_json
            }

    def odom_callback(self, msg):
        with self.lock:
            # Store the latest pose in attribute
            position = msg.pose.pose.position
            velocity = msg.twist.twist.linear
            orientation = msg.pose.pose.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = self.euler_from_quaternion(orientation_list)
            theta = yaw
            overall_velocity = math.sqrt(velocity.x**2 + velocity.y**2)
            self.current_position = {
                'position_x': position.x,
                'position_y': position.y,
                'velocity_x': overall_velocity,
                # Skip the z axis
                'theta': theta}

    def do_task(self):
        with self.lock:
            # Store the latest scan and pose in Redis
            if not self.current_scan or not self.current_position:
                self.get_logger().info('No data found in Redis scan or pose')
                return
            map_to_write = {
                'position': json.dumps((self.current_position['position_x'], self.current_position['position_y'])),
                'velocity': self.current_position['velocity_x'],
                'theta': self.current_position['theta'],
                'scan': json.dumps({
                    'angle_min': self.current_scan['angle_min'],
                    'angle_max': self.current_scan['angle_max'],
                    'angle_increment': self.current_scan['angle_increment'],
                    'time_increment': self.current_scan['time_increment'],
                    'scan_time': self.current_scan['scan_time'],
                    'range_min': self.current_scan['range_min'],
                    'range_max': self.current_scan['range_max'],
                    'ranges': json.loads(self.current_scan['ranges']),
                    'intensities': json.loads(self.current_scan['intensities'])
                })
            }
            self.get_logger().info(f'Writing to Redis: {map_to_write}')
            self.redis_client.hset(f'current_map_{self.namespace}', mapping=map_to_write)
            self.finished_publisher.publish(Bool(data=True))
            self.get_logger().info('Monitor finished publishing')
        

def main(args=None):
    rclpy.init(args=args)
    monitor = Monitor()
    rclpy.spin(monitor)
    rclpy.shutdown()
