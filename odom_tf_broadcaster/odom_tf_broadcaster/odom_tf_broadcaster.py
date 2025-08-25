import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFBroadcaster(Node):
    """
    Subscribes to nav_msgs/Odometry and publishes TF: <odom_frame> -> <base_frame>.
    Default frames are namespaced-safe; override via parameters.
    """
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # Parameters (per-robot via namespace or explicit values)
        self.declare_parameter('odom_topic', 'odom')                   # e.g. /tb1/odom
        self.declare_parameter('odom_frame', 'odom')                   # e.g. tb1/odom
        self.declare_parameter('base_frame', 'base_footprint')         # e.g. tb1/base_footprint

        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 50)

        self.get_logger().info(
            f'Bridging TF: {self.odom_frame} -> {self.base_frame} from topic "{odom_topic}"'
        )

    def _odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
