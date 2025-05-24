import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import TransformBroadcaster
import tf_transformations


class DetectPipe(Node):
    def __init__(self):
        super().__init__("detect_pipe")
        self.color_subscription = self.create_subscription(
            Image, "/Gen3/kinova_color/image_color", self.image_callback, 10
        )
        self.depth_subscription = self.create_subscription(
            Image, "/Gen3/kinova_depth/image", self.depth_callback, 10
        )

        self.bridge = CvBridge()
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.color_circles = None
        self.depth_circles = None

        self.color_hough_dp = 1.0
        self.color_hough_min_dist = 10
        self.color_hough_param1 = 80
        self.color_hough_param2 = 50
        self.color_min_radius = 5
        self.color_max_radius = 50

        # Depth Hough parameters (may need tuning)
        self.depth_hough_dp = 1.0
        self.depth_hough_min_dist = 10
        self.depth_hough_param1 = 80
        self.depth_hough_param2 = 50
        self.depth_min_radius = 0
        self.depth_max_radius = 50

        # Scaling factors (based on actual resolutions)
        self.depth_width = 640
        self.depth_height = 480
        self.color_width = 640
        self.color_height = 480
        self.scale_x = self.depth_width / self.color_width
        self.scale_y = self.depth_height / self.color_height

        # Dynamic offsets
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.max_offset_distance = 100.0  # Max pixel distance for matching circles

        # TF setup
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.base_frame = "base_link"
        self.camera_frame = "camera_link"

        self.get_logger().info("DetectPipe node has been started.")

    def find_matching_circle(self, x_color, y_color, r_color):
        """Find the closest depth circle to a color circle."""
        if self.depth_circles is None:
            return None, 0.0, 0.0

        min_distance = float("inf")
        best_match = None
        offset_x, offset_y = 0.0, 0.0

        for x_depth, y_depth, r_depth in self.depth_circles:
            distance = np.sqrt((x_depth - x_color) ** 2 + (y_depth - y_color) ** 2)
            radius_diff = abs(r_depth - r_color)

            # Match based on proximity and similar radius
            if (
                distance < min_distance
                and distance < self.max_offset_distance
                and radius_diff < 10
            ):
                min_distance = distance
                best_match = (x_depth, y_depth, r_depth)
                offset_x = x_depth - x_color
                offset_y = y_depth - y_color

        return best_match, offset_x, offset_y

    def image_callback(self, msg):
        self.latest_color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = self.latest_color_frame.copy()

        self.get_logger().debug(f"Color image shape: {frame.shape}")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        self.color_circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=self.color_hough_dp,
            minDist=self.color_hough_min_dist,
            param1=self.color_hough_param1,
            param2=self.color_hough_param2,
            minRadius=self.color_min_radius,
            maxRadius=self.color_max_radius,
        )

        if self.color_circles is not None:
            self.color_circles = np.round(self.color_circles[0, :]).astype("int")
            for x, y, r in self.color_circles:
                x_centered = x - (self.color_width // 2)
                y_centered = y - (self.color_height // 2)

                # Initial coordinates
                x_scaled = float(x * self.scale_x)
                y_scaled = float(y * self.scale_y)
                r_scaled = float(r * self.scale_x)

                # Find matching depth circle and compute offsets
                _, offset_x, offset_y = self.find_matching_circle(
                    x_scaled, y_scaled, r_scaled
                )
                self.offset_x, self.offset_y = offset_x, offset_y

                # Apply offsets
                x_depth = x_scaled + self.offset_x
                y_depth = y_scaled + self.offset_y

                self.get_logger().info(
                    f"Circle original: (x={x}, y={y}), depth coords: (x_depth={x_depth:.2f}, y_depth={y_depth:.2f}), "
                    f"offsets: ({self.offset_x:.2f}, {self.offset_y:.2f})"
                )

                # Draw on color image
                cv2.circle(
                    frame, (int(x_depth), int(y_depth)), int(r_scaled), (0, 255, 0), 4
                )
                cv2.rectangle(
                    frame,
                    (int(x_depth - 5), int(y_depth - 5)),
                    (int(x_depth + 5), int(y_depth + 5)),
                    (0, 128, 255),
                    -1,
                )

                # Check depth
                if self.latest_depth_frame is not None:
                    if (
                        0 <= int(y_depth) < self.depth_height
                        and 0 <= int(x_depth) < self.depth_width
                    ):
                        y_min = max(0, int(y_depth - r_scaled))
                        y_max = min(self.depth_height, int(y_depth + r_scaled))
                        x_min = max(0, int(x_depth - r_scaled))
                        x_max = min(self.depth_width, int(x_depth + r_scaled))
                        depth_roi = self.latest_depth_frame[y_min:y_max, x_min:x_max]

                        if depth_roi.size == 0:
                            self.get_logger().warn(
                                f"Empty depth ROI at ({x_depth}, {y_depth})"
                            )
                            continue

                        valid_depths = depth_roi[depth_roi > 0]
                        if valid_depths.size == 0:
                            self.get_logger().warn(
                                f"No valid depth values in ROI at ({x_depth}, {y_depth})"
                            )
                            continue

                        avg_depth = np.mean(valid_depths)
                        distance_meters = avg_depth / 1000.0  # Convert mm to meters

                        if distance_meters < 1.0:
                            self.get_logger().info(
                                f"Circle at ({x_centered}, {y_centered}) [depth coords: ({x_depth}, {y_depth})], "
                                f"radius {r} [scaled: {r_scaled}], "
                                f"avg distance: {distance_meters:.2f} meters"
                            )

                        else:
                            self.get_logger().debug(
                                f"Circle at ({x_centered}, {y_centered}) with avg distance {distance_meters:.2f} meters ignored (>= 1 meter)"
                            )
                    else:
                        self.get_logger().warn(
                            f"Scaled circle at ({x_depth}, {y_depth}) out of depth bounds ({self.depth_width}x{self.depth_height})"
                        )
                else:
                    self.get_logger().info(
                        f"Circle at ({x_centered}, {y_centered}) with radius {r} (no depth yet)"
                    )

        else:
            self.get_logger().debug("No circles detected in color image")

        if self.latest_depth_frame is not None:
            depth_display = cv2.normalize(
                self.latest_depth_frame, None, 0, 255, cv2.NORM_MINMAX
            )
            depth_display = cv2.convertScaleAbs(depth_display)
            depth_display = cv2.cvtColor(depth_display, cv2.COLOR_GRAY2BGR)
            blended = cv2.addWeighted(frame, 0.5, depth_display, 0.5, 0.0)
            # cv2.imshow("Blended Color and Depth", blended)
        # cv2.imshow("Color - Detected Circles", frame)
        # cv2.waitKey(1)

    def depth_callback(self, msg):
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, "32FC1")

        if self.latest_depth_frame is None or self.latest_depth_frame.size == 0:
            self.get_logger().error("Error: depth image is empty")
            return

        self.get_logger().debug(f"Depth image shape: {self.latest_depth_frame.shape}")

        # depth image for circle detection
        depth_norm = cv2.normalize(
            self.latest_depth_frame, None, 0, 255, cv2.NORM_MINMAX
        )
        depth_gray = cv2.convertScaleAbs(depth_norm)
        depth_gray = cv2.GaussianBlur(depth_gray, (5, 5), 0)

        self.depth_circles = cv2.HoughCircles(
            depth_gray,
            cv2.HOUGH_GRADIENT,
            dp=self.depth_hough_dp,
            minDist=self.depth_hough_min_dist,
            param1=self.depth_hough_param1,
            param2=self.depth_hough_param2,
            minRadius=self.depth_min_radius,
            maxRadius=self.depth_max_radius,
        )

        depth_display = cv2.cvtColor(depth_gray, cv2.COLOR_GRAY2BGR)

        if self.depth_circles is not None:
            self.depth_circles = np.round(self.depth_circles[0, :]).astype("int")
            for x, y, r in self.depth_circles:
                cv2.circle(depth_display, (x, y), r, (255, 0, 0), 4)
                cv2.rectangle(
                    depth_display,
                    (x - 5, y - 5),
                    (x + 5, y + 5),
                    (255, 128, 0),
                    -1,
                )
        else:
            self.get_logger().debug("No circles detected in depth image")

        if self.color_circles is not None:
            for x, y, r in self.color_circles:
                x_scaled = float(x * self.scale_x)
                y_scaled = float(y * self.scale_y)
                r_scaled = float(r * self.scale_x)

                x_depth = x_scaled + self.offset_x
                y_depth = y_scaled + self.offset_y

                x_depth_int = int(x_depth)
                y_depth_int = int(y_depth)
                r_scaled_int = int(r_scaled)

                cv2.circle(
                    depth_display,
                    (x_depth_int, y_depth_int),
                    r_scaled_int,
                    (0, 255, 0),
                    4,
                )
                cv2.rectangle(
                    depth_display,
                    (x_depth_int - 5, y_depth_int - 5),
                    (x_depth_int + 5, y_depth_int + 5),
                    (0, 128, 255),
                    -1,
                )
                self.broadcast_pipe_transform(
                    x_depth, y_depth, self.latest_depth_frame[y_depth_int, x_depth_int]
                )

        # cv2.imshow("Depth - Overlay", depth_display)
        # cv2.waitKey(1)

    def broadcast_pipe_transform(self, cX, cY, depth):
        fx = 772.5
        fy = 772.5
        cx = 320.0
        cy = 240.0

        X = (cX - cx) * depth / fx
        Y = (cY - cy) * depth / fy
        Z = depth

        self.get_logger().info(
            f"pipe in kinova vision: x={X:.3f}, y={Y:.3f}, z={Z:.3f}"
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

            base_pos = np.dot(rotation_matrix, cam_pos) + translation

            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = "base_link"
            tf_msg.child_frame_id = f"pipe"
            tf_msg.transform.translation.x = base_pos[0]
            tf_msg.transform.translation.y = base_pos[1]
            tf_msg.transform.translation.z = base_pos[2]
            tf_msg.transform.rotation.x = 0.5
            tf_msg.transform.rotation.y = 0.5
            tf_msg.transform.rotation.z = -0.5
            tf_msg.transform.rotation.w = -0.5

            self.tf_broadcaster.sendTransform(tf_msg)
            self.get_logger().info(
                f"[TF] pipe in base_link: x={base_pos[0]:.3f}, "
                f"y={base_pos[1]:.3f}, z={base_pos[2]:.3f}"
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in TF broadcast: {e}")


def main(args=None):
    try:
        rclpy.init(args=args)
        detect_pipe = DetectPipe()
        rclpy.spin(detect_pipe)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            detect_pipe.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
