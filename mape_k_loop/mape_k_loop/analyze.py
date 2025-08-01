import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
import rclpy
from rclpy.node import Node
import redis
import json
import numpy as np
from fractions import Fraction
from threading import Lock
from std_msgs.msg import String, Bool
from mape_k_interfaces.srv import CheckAnomaly
from mape_k_loop.base_slave import BaseSlave

try:
    from .lidarocclusion.masks import BoolLidarMask
    from .lidarocclusion.sliding_lidar_masks import sliding_lidar_mask
except (ValueError, ImportError):
    from mape_k_loop.lidarocclusion.masks import BoolLidarMask
    from mape_k_loop.lidarocclusion.sliding_lidar_masks import sliding_lidar_mask


WINDOW_SIZE = 5
SIMULATED_OCCLUSION = (0,Fraction(1, 4))
def scan_to_mask(scan):
    ranges = np.array(scan['ranges'])
    return BoolLidarMask(
        (ranges != np.inf) & (ranges != -np.inf),
        base_angle=Fraction(2, len(ranges))
    )

class Analyze(BaseSlave):
    def __init__(self):
        super().__init__(f'analyze')
        self.namespace = self.get_namespace().strip('/')
        self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
        try:
            self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)    
            if not self.redis_client.ping():
                self.get_logger().info('Connected to Redis server')
        except ConnectionError as e:
            self.get_logger().error(f'Failed to connect to Redis server: {e}')
            raise RuntimeError("Redis unavailable, aborting Analyze node") from e
        self.get_logger().info('Analyze node started')
        self.scans = []
        self.anomaly = False
        self.get_logger().info(f'Current_namespace: {self.namespace}')
        self.anomaly_srv = self.create_service(CheckAnomaly, f'/{self.namespace}/check_anomaly', self.check_anomaly_callback)
        self.anomaly_init = 0

        def raw_lidar_masks():
            for scan in self.scans:
                self.get_logger().info(f'scan: {type(scan)}')
                yield scan_to_mask(scan)

        self._sliding_lidar_mask = sliding_lidar_mask(
                raw_lidar_masks(),
                window_size=5,
                cutoff=0.5
            )
        self.anomaly_topic = self.create_publisher(String, f'{self.namespace}/anomaly', self.qos)
        
    def check_anomaly_callback (self, _, response):
        response.anomaly = self.anomaly
        return response

    def do_task(self):
        # Read Latest Scan
        current_scan = self.redis_client.hgetall(f'current_map_{self.namespace}')
        if not current_scan:
            self.get_logger().warn('No scan data found in Redis')
            return False
        lidar_info  = json.loads(current_scan['scan'])
        
        # Convert the scan to a mask
        self.get_logger().info(f"Current_scan: {len(lidar_info)}")
    
        self.scans.append(lidar_info)
        if len(self.scans) < WINDOW_SIZE:
            self.get_logger().info(f"Waiting for {WINDOW_SIZE} scans (have {len(self.scans)})")
            return False
        sliding_mask = next(self._sliding_lidar_mask)
        simulated_occlusion = None        
        ignore_lidar_region = BoolLidarMask(
            [(3 * np.pi / 4, 5 * np.pi / 4)],
            sliding_mask.base_angle,
        )

        sliding_mask  = sliding_mask | ignore_lidar_region
        if simulated_occlusion:
            sliding_mask = sliding_mask & simulated_occlusion
        else:
            sliding_mask = sliding_mask

        if sliding_mask._values.all():
            self.anomaly = False
            self.anomaly_init += 1
        else:
            self.anomaly = True
            self.get_logger().info('Anomaly detected!')
            self.redis_client.hset(f'anomaly_{self.namespace}', mapping={
                'anomaly': json.dumps(True),
                'scan': json.dumps(lidar_info)
            })
            self.anomaly_topic.publish(String(data=json.dumps(lidar_info)))
        
        return True
            
        
            

def main(args=None):
    rclpy.init(args=args)
    plan = Analyze()
    rclpy.spin(plan)
    rclpy.shutdown()