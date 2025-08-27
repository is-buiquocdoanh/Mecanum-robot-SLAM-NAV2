import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math

def quaternion_to_yaw(q):
    import math
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class GoalErrorNode(Node):
    def __init__(self):
        super().__init__('goal_error_node')
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_pose = None
        self.robot_pose = None

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def send_goal(self, goal_pose):
        self.goal_pose = goal_pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.client.wait_for_server()
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal succeeded!')
        if self.goal_pose and self.robot_pose:
            dx = self.goal_pose.pose.position.x - self.robot_pose.position.x
            dy = self.goal_pose.pose.position.y - self.robot_pose.position.y
            dist_error = math.sqrt(dx*dx + dy*dy)

            yaw_goal = quaternion_to_yaw(self.goal_pose.pose.orientation)
            yaw_robot = quaternion_to_yaw(self.robot_pose.orientation)
            ang_error = abs(yaw_goal - yaw_robot)

            self.get_logger().info(f"Distance error: {dist_error:.3f} m")
            self.get_logger().info(f"Angle error: {ang_error:.3f} rad")

def main():
    rclpy.init()
    node = GoalErrorNode()
    # Ví dụ đặt goal tại (2, 1)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 2.0
    pose.pose.position.y = 1.0
    pose.pose.orientation.w = 1.0
    node.send_goal(pose)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
