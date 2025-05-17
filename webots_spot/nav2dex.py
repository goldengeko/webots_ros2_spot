import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavToDex(Node):
    def __init__(self):
        super().__init__("nav_to_dex")
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.pose_sub = self.create_subscription(
            PoseStamped, "dex_board_pose", self.pose_callback, 10
        )
        self.get_logger().info("NavToDex Node Initialized")

    def pose_callback(self, msg):
        # if msg.header.frame_id != "map":
        #     self.get_logger().error(f"Received pose in unsupported frame: {msg.header.frame_id}")
        #     return

        # Create goal pose 0.5m away from dex_board_green
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "odom"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = msg.pose.position.x - 1.5  # 0.5m offset
        goal_msg.pose.pose.position.y = msg.pose.position.y
        goal_msg.pose.pose.position.z = 0.0  # Ground navigation
        goal_msg.pose.pose.orientation.w = 1.0  # Keep same orientation

        self.get_logger().info(
            f"Sending goal: x={goal_msg.pose.pose.position.x:.3f}, "
            f"y={goal_msg.pose.pose.position.y:.3f}, z={goal_msg.pose.pose.position.z:.3f}"
        )

        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available after waiting")
            return

        # Send goal
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Goal sent successfully")
        else:
            self.get_logger().error("Failed to send goal")


def main(args=None):
    rclpy.init(args=args)
    node = NavToDex()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
