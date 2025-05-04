import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '.'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
import rclpy
from rclpy.node import Node
import redis
import json
import signal
import numpy as np
from fractions import Fraction
from threading import Lock
try:
    from .lidarocclusion.masks import BoolLidarMask
    from .lidarocclusion.sliding_lidar_masks import sliding_lidar_mask
except (ValueError, ImportError):
    from lidarocclusion.masks import BoolLidarMask
    from lidarocclusion.sliding_lidar_masks import sliding_lidar_mask



SIMULATED_OCCLUSION = (0,Fraction(1, 4))
def scan_to_mask(scan):
    ranges = np.array(scan['ranges'])
    return BoolLidarMask(
        (ranges != np.inf) & (ranges != -np.inf),
        base_angle=Fraction(2, len(ranges))
    )

class Analyze(Node):
    def __init__(self):
        super().__init__(f'analyze')
        
        self.namespace = self.get_namespace().strip('/')
        self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
        if self.redis_client.ping():
            self.get_logger().info('Connected to Redis server')
        else:
            # If the database connection fails, set up a signal handler to stop execution
            self.get_logger().error('Failed to connect to Redis server')
            def stop_execution(signal, frame):
                self.get_logger().info('Stopping execution...')
                rclpy.shutdown()
                exit(0)

            signal.signal(signal.SIGINT, stop_execution)
        self.reader = self.create_timer(0.5,  self.read_from_db)
        self.get_logger().info('Analyze node started')
        self.scans = []
        self.anomaly = False
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
        self.scans_mutex = Lock()
        
        

    # Add a timer callback to periodically read from database
    def read_from_db(self):
        current_scan = self.redis_client.hgetall(f'current_map_{self.namespace}')
        if not current_scan:
            self.get_logger().warn('No scan data found in Redis')
            return
        lidar_info  = json.loads(current_scan['scan'])
        
        # Convert the scan to a mask
        
        with self.scans_mutex:
            self.scans.append(lidar_info)
            # Use the sliding_lidar_mask function to get the sliding mask
            sliding_mask = next(self._sliding_lidar_mask)
            self.get_logger().info(f'sliding_mask: {sliding_mask}')
            self.get_logger().info(f'sliding_mask: {type(sliding_mask._values)}')
            simulated_occlusion = None
            if self.anomaly_init >= 15:
                simulated_occlusion = BoolLidarMask(
                    np.zeros_like(lidar_info.values, dtype=bool),
                    base_angle= Fraction(1,4) # 1/4 pi radians
                )
                self.get_logger().info(f'simulated_occlusion: {simulated_occlusion._values.shape}')
            
            ignore_lidar_region = BoolLidarMask(
                [(3 * np.pi / 4, 5 * np.pi / 4)],
                sliding_mask.base_angle,
            )

            sliding_mask  = sliding_mask | ignore_lidar_region
            if simulated_occlusion:
                sliding_mask = sliding_mask | simulated_occlusion
            else:
                sliding_mask = sliding_mask

            if sliding_mask._values.all():
                self.anomaly = False
                self.anomaly_init += 1
            else:
                self.get_logger().info(f'sliding_mask: {sliding_mask._values}')
                self.anomaly = True
                self.get_logger().info('Anomaly detected')

            
        
            

def main(args=None):
    rclpy.init(args=args)
    plan = Analyze()
    rclpy.spin(plan)
    rclpy.shutdown()