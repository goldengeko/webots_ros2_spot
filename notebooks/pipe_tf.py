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
from tf2_ros import Buffer, TransformListener
import tf_transformations
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class CircleDetectorTF(Node):
    def __init__(self):
        super().__init__("circle_detector_tf")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        self.camera_frame = "camera_link"  # Change this to your camera frame name

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.circle_subscription = self.create_subscription(
            String, "/detected_pipe/center", self.transform_callback, 10
        )

    def broadcast_pipe_transform(self, cX, cY, depth, color):
        fx = 772.5
        fy = 772.5
        cx = 320.0
        cy = 240.0

        X = (cX - cx) * depth / fx
        Y = (cY - cy) * depth / fy
        Z = depth

        self.get_logger().info(
            f"dex_board in kinova vision: x={X:.3f}, y={Y:.3f}, z={Z:.3f}"
        )

        cam_pos = np.array([X, Y, Z], dtype=float)

        try:
            if not self.tf_buffer.can_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            ):
                self.get_logger().error(
                    "Transform from kinova vision to base_link not available"
                )
                return

            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, rclpy.time.Time()
            )
            self.get_logger().info(
                f"Transform: translation=({transform.transform.translation.x:.3f}, "
                f"{transform.transform.translation.y:.3f}, {transform.transform.translation.z:.3f}), "
                f"rotation=({transform.transform.rotation.x:.3f}, {transform.transform.rotation.y:.3f}, "
                f"{transform.transform.rotation.z:.3f}, {transform.transform.rotation.w:.3f})"
            )

            translation = np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ],
                dtype=float,
            )

            quaternion = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]

            rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]

            # Step 5: Transform position to base frame
            base_pos = np.dot(rotation_matrix, cam_pos) + translation

            # Step 6: Broadcast TF
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = "base_link"
            tf_msg.child_frame_id = f"dex_board_{color}"
            tf_msg.transform.translation.x = base_pos[0]
            tf_msg.transform.translation.y = base_pos[1]
            tf_msg.transform.translation.z = base_pos[2]
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = 0.0
            tf_msg.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(tf_msg)
            self.get_logger().info(
                f"[TF] dex_board in base_link: x={base_pos[0]:.3f}, "
                f"y={base_pos[1]:.3f}, z={base_pos[2]:.3f}"
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in TF broadcast: {e}")


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
