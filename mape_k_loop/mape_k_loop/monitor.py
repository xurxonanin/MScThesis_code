import rclpy
from rclpy.node import Node
import redis
from sensor_msgs.msg import LaserScan
# from lidarocclusion.masks import BoolLidarMask
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
import json

class Monitor(Node):
    def __init__(self):
        super().__init__(f'monitor')
        self.laser_sub = self.create_subscription(LaserScan, 
            'scan',
            self.laser_callback, 
            2
        )    

        self.odom_sub = self.create_subscription(Odometry,
            'odom',
            self.odom_callback, 
            2
        )
        self.get_logger().info(f'/monitor node started')

        self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
        if self.redis_client.ping():
            self.get_logger().info('Connected to Redis server')
        else:
            self.get_logger().error('Failed to connect to Redis server')


    def euler_from_quaternion(self, quaternion):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        """
        r = R.from_quat(quaternion)
        # Convert to radians
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        return roll, pitch, yaw

    def laser_callback(self, msg):

        ranges_json = json.dumps(list(msg.ranges))
        intensities_json = json.dumps(list(msg.intensities))
        self.redis_client.hset('latest_scan', mapping = {
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges': ranges_json,
            'intensities': intensities_json
        })
        self.get_logger().info(f'Published latest_scan to Redis: {msg.header.stamp.sec}')

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = self.euler_from_quaternion(orientation_list)
        theta = yaw
        self.get_logger().info(f'Position: {position.x}, {position.y}')
        self.redis_client.hset('latest_pose', mapping={
            'position_x': position.x,
            'position_y': position.y,
            'velocity_x': velocity.x, # are we interested on the velocity of the robot?
            'velocity_y': velocity.y,    
            # Skip the z axis
            'theta': theta})

    

def main(args=None):
    rclpy.init(args=args)
    monitor = Monitor()
    rclpy.spin(monitor)
    rclpy.shutdown()
