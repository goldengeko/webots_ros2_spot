#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class SetReadyPosition(Node):
    def __init__(self):
        super().__init__("set_ready_position")
        self.clock_msg_count = 0

        # Action client for kinova_joint_trajectory_controller
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/kinova_joint_trajectory_controller/follow_joint_trajectory",
        )

        # Subscribe to clock to ensure simulation is ready
        self.clock_sub = self.create_subscription(
            Clock, "/clock", self.clock_callback, 1
        )

    def clock_callback(self, msg):
        self.clock_msg_count += 1

    def send_ready_position(self):
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Joint trajectory action server not available")
            return False

        # Define ready position (non-singular, slightly extended)
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
        point = JointTrajectoryPoint()
        point.positions = [
            0.0,  # joint_1: Neutral
            0.8,  # joint_2: Slightly extended
            -2.3,  # joint_3: Mid-range
            0.0,  # joint_4: Neutral
            1.57,  # joint_5: Slight bend
            1.57,  # joint_6: Neutral
        ]
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 3  # Move over 3 seconds
        goal_msg.trajectory.points.append(point)

        self.get_logger().info("Sending ready position goal")
        self.arm_client.send_goal_async(goal_msg)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SetReadyPosition()

    # Wait for simulation clock to stabilize
    while node.clock_msg_count < 40:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Send ready position and exit
    success = node.send_ready_position()
    if success:
        node.get_logger().info("Ready position sent successfully")
    else:
        node.get_logger().error("Failed to send ready position")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
