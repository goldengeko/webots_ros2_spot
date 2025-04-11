import rclpy
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math


def main():
    rclpy.init()
    node = rclpy.create_node("retract_manipulator_node")

    def increment_count(_):
        nonlocal clock_msg_count
        clock_msg_count += 1

    clock_msg_count = 0

    node.create_subscription(Clock, "/clock", increment_count, 1)

    # Create an action client for the gen3_joint_trajectory_controller
    arm_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/joint_trajectory_controller/follow_joint_trajectory",
    )

    # Create an action client for the robotiq_gripper_controller
    gripper_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/robotiq_gripper_controller/follow_joint_trajectory",
    )

    arm_client.wait_for_server()
    gripper_client.wait_for_server()

    # Wait for simulation clock to initiate
    while clock_msg_count < 40:
        rclpy.spin_once(node)

    # Create a goal request to set arm joint positions
    arm_goal_msg = FollowJointTrajectory.Goal()
    arm_goal_msg.trajectory.joint_names = [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    ]
    arm_point = JointTrajectoryPoint()
    arm_point.positions = [
        0.0,
        math.radians(180),
        math.radians(180),
        0.0,
        0.0,
        0.0,
    ]
    arm_point.time_from_start.sec = 5  # Set a duration for the motion
    arm_goal_msg.trajectory.points.append(arm_point)

    # Create a goal request to set gripper joint positions
    gripper_goal_msg = FollowJointTrajectory.Goal()
    gripper_goal_msg.trajectory.joint_names = [
        "gripper_left_finger_joint",
        "gripper_right_finger_joint",
    ]
    gripper_point = JointTrajectoryPoint()
    gripper_point.positions = [0.0, 0.0]  # Fully close the gripper
    gripper_point.time_from_start.sec = 2  # Set a duration for the motion
    gripper_goal_msg.trajectory.points.append(gripper_point)

    # Send action goals
    arm_client.send_goal_async(arm_goal_msg)
    gripper_client.send_goal_async(gripper_goal_msg)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
