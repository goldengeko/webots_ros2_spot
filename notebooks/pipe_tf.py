import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from tf2_ros import Buffer, TransformListener  # Import for TF lookup


class CircleDetectorTF(Node):
    def __init__(self):
        super().__init__("circle_detector_tf")

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        self.camera_frame = "camera_color_frame"

        # TF Buffer and Listener to get camera orientation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.circle_subscription = self.create_subscription(
            String, "/detected_pipe/center", self.transform_callback, 10
        )

    def transform_callback(self, msg):
        try:
            data = msg.data.strip().split(", ")
            x_center = float(data[0].split(":")[1])
            y_center = float(data[1].split(":")[1])
            distance = float(data[2].split(":")[1].split(" ")[0])
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse message: {msg.data}")
            return

        z = distance
        x = x_center / 1000
        y = y_center / 1000

        # Get the camera's orientation from the TF tree
        try:
            camera_transform = self.tf_buffer.lookup_transform(
                "camera_link",
                self.camera_frame,
                rclpy.time.Time(),
            )
            rotation = camera_transform.transform.rotation
        except Exception as e:
            self.get_logger().error(f"Failed to get camera orientation: {str(e)}")
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = "detected_circle"

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.x = rotation.x
        transform.transform.rotation.y = rotation.y
        transform.transform.rotation.z = rotation.z
        transform.transform.rotation.w = rotation.w

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"Broadcasted transform: x={x:.2f}, y={y:.2f}, z={z:.2f}, "
            f"rotation=({rotation.x:.2f}, {rotation.y:.2f}, {rotation.z:.2f}, {rotation.w:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CircleDetectorTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
