import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros


class CircleDetectorTF(Node):
    def __init__(self):
        super().__init__("circle_detector_tf")

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        # Camera frame and odom frame
        self.camera_frame = "camera_link"
        self.odom_frame = "odom"

        # Initialize tf2 Buffer and Listener for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera intrinsics (from /camera_info, you can refine later)
        self.fx = 615.0
        self.fy = 615.0
        self.cx = 320.0
        self.cy = 240.0

        # Image subscribers
        self.subscription_color = self.create_subscription(
            Image, "/Gen3/kinova_color/image_color", self.image_callback, 10
        )

        self.subscription_depth = self.create_subscription(
            Image, "/Gen3/kinova_depth/image", self.depth_callback, 10
        )

        self.color_image = None
        self.depth_image = None

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.try_process()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )
        self.try_process()

    def try_process(self):
        if self.color_image is None or self.depth_image is None:
            return

        gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        # Circle detection
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=50,
            param2=30,
            minRadius=10,
            maxRadius=100,
        )

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")

            for i, (x, y, r) in enumerate(circles):
                # Get depth around center
                depth_patch = self.depth_image[y - 2 : y + 3, x - 2 : x + 3]
                valid = depth_patch[depth_patch > 0]

                if valid.size == 0:
                    continue

                avg_depth = np.mean(valid)
                z = avg_depth / 1000.0  # Convert mm to meters

                # Convert to 3D
                x3d = (x - self.cx) * z / self.fx
                y3d = (y - self.cy) * z / self.fy

                # Transform from camera frame to odom frame
                self.publish_transform_to_odom(x3d, y3d, z, i)

                # Visualization (optional)
                cv2.circle(self.color_image, (x, y), r, (0, 255, 0), 2)
                cv2.circle(self.color_image, (x, y), 2, (0, 0, 255), 3)
                cv2.putText(
                    self.color_image,
                    f"circle_{i}",
                    (x - 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                )

        # Show debug window
        cv2.imshow("Circles", self.color_image)
        cv2.waitKey(1)

    def publish_transform_to_odom(self, x, y, z, index):
        try:
            # Get the transform from camera_frame to odom frame
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.camera_frame, rclpy.time.Time()
            )

            # Transform circle coordinates from camera frame to odom frame
            point_camera = tf2_geometry_msgs.TransformStamped()
            point_camera.header.frame_id = self.camera_frame
            point_camera.child_frame_id = f"circle_{index}"
            point_camera.transform.translation.x = x
            point_camera.transform.translation.y = y
            point_camera.transform.translation.z = z

            # Apply the transformation
            transformed_point = tf2_geometry_msgs.do_transform_translation(
                point_camera, transform
            )

            # Publish the transformed point as a TF to odom
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = f"circle_{index}"

            t.transform.translation.x = transformed_point.transform.translation.x
            t.transform.translation.y = transformed_point.transform.translation.y
            t.transform.translation.z = transformed_point.transform.translation.z
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")


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
