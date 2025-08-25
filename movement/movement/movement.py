import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math, random, time

def make_pose(x, y , yaw_deg, frame_id='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    yaw_rad = math.radians(yaw_deg)

    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose

class Movement(Node):
    def __init__(self):
        super().__init__('movement')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.bounds = (-1.5, 1.5, -1.5, 1.5)  # (x_min, x_max, y_min, y_max)
        self.send_random_goal()

    def send_random_goal(self):
        min_x, max_x, min_y, max_y = self.bounds
        gx = random.uniform(min_x, max_x)
        gy = random.uniform(min_y, max_y)
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(gx, gy, yaw_deg = 0.0)
        self.get_logger().info(f'Sending goal: {gx:.2f}, {gy:.2f}')
        self.client.wait_for_server()
        self.client.send_goal_async(goal)



def main(args=None):
    rclpy.init(args=args)
    movement_node = Movement()
    rclpy.spin(movement_node)
    movement_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()