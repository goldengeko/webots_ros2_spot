import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from webots_spot_msgs.srv import SpotMotion
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class SitServiceClient(Node):
    def __init__(self):
        super().__init__("sit_service_client")
        self.get_logger().info("Initializing SitServiceClient...")
        self.sit_cli = self.create_client(SpotMotion, "/Spot/lie_down")

        while not self.sit_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for sit service...")

        self.req = SpotMotion.Request()
        self.req.override = True

        self.future = self.sit_cli.call_async(self.req)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Sit service responded: {response.answer}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


class NavToDex(Node):
    def __init__(self):
        super().__init__("nav_to_dex")
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("NavToDex Node Initialized")

        # Wait for transform
        while not self.tf_buffer.can_transform(
            "base_link", "linear_front", Time(), Duration(seconds=0.05)
        ):
            self.get_logger().info(
                "Waiting for transform from linear_front to base_link..."
            )
            rclpy.spin_once(self, timeout_sec=0.1)

        try:
            t = self.tf_buffer.lookup_transform(
                "base_link", "linear_front", Time(), Duration(seconds=0.05)
            )

            self.goal_msg = NavigateToPose.Goal()
            self.goal_msg.pose = PoseStamped()
            self.goal_msg.pose.header.frame_id = "map"
            self.goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_msg.pose.pose.position.x = t.transform.translation.x - 0.8
            self.goal_msg.pose.pose.position.y = t.transform.translation.y
            self.goal_msg.pose.pose.position.z = 0.0
            self.goal_msg.pose.pose.orientation.w = 1.0

        except Exception as e:
            self.get_logger().error(f"Could not transform: {e}")
            rclpy.shutdown()

    def send_goal(self):
        self.get_logger().info("Waiting for action server...")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available.")
            rclpy.shutdown()
            return

        self._send_goal_future = self.action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted.")
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("Navigation complete, cancelling action before sitting.")
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.after_cancel_callback)

    def after_cancel_callback(self, future):
        self.get_logger().info("Navigation action cancelled, triggering sit behavior.")
        sit_node = SitServiceClient()
        executor.add_node(sit_node)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback: Current distance to goal: {feedback.distance_remaining:.2f}m"
        )


def main(args=None):
    rclpy.init(args=args)

    global executor
    executor = MultiThreadedExecutor()

    nav_to_dex = NavToDex()
    executor.add_node(nav_to_dex)

    nav_to_dex.send_goal()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        nav_to_dex.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
