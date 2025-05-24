import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import TransformBroadcaster
import tf_transformations


green = False


class DexBoardDetector(Node):
    def __init__(self):
        super().__init__("dex_board_detector")
        self.image_sub = self.create_subscription(
            Image, "Gen3/kinova_color/image_color", self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "Gen3/kinova_depth/image", self.depth_callback, 10
        )
        self.dex_board_pub = self.create_publisher(String, "dex_board_position", 10)
        self.bridge = CvBridge()
        self.latest_depth_image = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.base_frame = "odom"
        self.camera_frame = "camera_link"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("dex_board Detector Node Initialized")

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="32FC1"
            )
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_green = np.array([40, 70, 70])
        upper_green = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

        lower_blue = np.array([100, 150, 70])
        upper_blue = np.array([140, 255, 255])
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        kernel = np.ones((5, 5), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

        green_contours, _ = cv2.findContours(
            green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        self.get_logger().info(f"Number of green regions: {len(green_contours)}")

        if green_contours:
            self.process_contours(cv_image, green_contours, "green")

        blue_contours, _ = cv2.findContours(
            blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        self.get_logger().info(f"Number of blue regions: {len(blue_contours)}")

        if blue_contours:
            # for contour in blue_contours:
            #     epsilon = 0.04 * cv2.arcLength(contour, True)
            #     approx = cv2.approxPolyDP(contour, epsilon, True)

            # if len(approx) == 4:
            self.process_contours(cv_image, blue_contours, "blue")

        # cv2.imshow("dex_board Detection", cv_image)
        # cv2.imshow("green_mask", green_mask)
        # cv2.imshow("blue_mask", blue_mask)
        # cv2.waitKey(1)

    def process_contours(self, cv_image, contours, color):
        combined_mask = np.zeros_like(cv_image[:, :, 0])
        cv2.drawContours(combined_mask, contours, -1, (255), thickness=cv2.FILLED)

        moments = cv2.moments(combined_mask)
        if moments["m00"] != 0:
            cX = int(moments["m10"] / moments["m00"])
            cY = int(moments["m01"] / moments["m00"])

            depth = None
            if self.latest_depth_image is not None:
                try:
                    depth = self.latest_depth_image[cY, cX]
                    self.get_logger().info(f"Raw depth at ({cX}, {cY}): {depth}")
                    if np.isnan(depth) or depth == 0:
                        depth = None
                except Exception as e:
                    self.get_logger().error(f"Depth fetch error: {e}")

            if depth is not None:
                self.broadcast_dex_board_transform(cX, cY, depth, color)

            x, y, w, h = cv2.boundingRect(combined_mask)
            color_bgr = (0, 255, 0) if color == "green" else (255, 0, 0)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), color_bgr, 2)
            cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)

            depth_str = f"{depth:.3f} m" if depth is not None else "invalid"
            msg_str = (
                f"{color.capitalize()} dex_board at: ({cX}, {cY}), Depth: {depth_str}"
            )
            self.get_logger().info(msg_str)
            dex_board_msg = String()
            dex_board_msg.data = msg_str
            self.dex_board_pub.publish(dex_board_msg)
        else:
            self.get_logger().info(f"Centroid undefined — zero area {color} dex_board.")

    def broadcast_dex_board_transform(self, cX, cY, depth, color):
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
                timeout=rclpy.duration.Duration(seconds=0.05),
            ):
                self.get_logger().error(
                    f"Transform from {self.camera_frame} to {self.base_frame} not available"
                )
                return

            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
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
            tf_msg.header.frame_id = self.base_frame
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
                f"[TF] dex_board in {self.base_frame}: x={base_pos[0]:.3f}, "
                f"y={base_pos[1]:.3f}, z={base_pos[2]:.3f}"
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in TF broadcast: {e}")


def main(args=None):
    rclpy.init(args=args)
    dex_board_detector = DexBoardDetector()
    try:
        rclpy.spin(dex_board_detector)
    except KeyboardInterrupt:
        pass
    finally:
        dex_board_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
