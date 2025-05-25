from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml


def generate_launch_description():
    home = os.path.expanduser("~")
    yaml_path = os.path.join(home, "dex_board_poses.yaml")
    dex_static_nodes = []
    detect_node = None

    # Always launch these static transforms
    static_nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "0",
                "0",
                "0",
                "0.5",
                "0.5",
                "-0.5",
                "-0.5",
                "dex_board_green",
                "linear_front",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "0.3",
                "0",
                "-0.08",
                "0",
                "0",
                "0",
                "1.0",
                "linear_front",
                "front_left",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "-0.3",
                "0",
                "-0.08",
                "0",
                "0",
                "0",
                "1.0",
                "linear_front",
                "front_right",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "0.15",
                "0",
                "-0.1",
                "0",
                "-0.4871745",
                "0.0",
                "-0.8733046",
                "linear_front",
                "angled_left",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "-0.15",
                "0",
                "-0.1",
                "0",
                "0.4871745",
                "0.0",
                "-0.8733046",
                "linear_front",
                "angled_right",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "0",
                "0",
                "0",
                "0.5",
                "0.5",
                "-0.5",
                "-0.5",
                "dex_board_blue",
                "omni_front",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "0.15",
                "0.15",
                "-0.05",
                "-0.3535534",
                "0.3535534",
                "-0.1464466",
                "0.8535534",
                "omni_front",
                "top_left",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "0.15",
                "-0.15",
                "-0.05",
                "0.3535534",
                "0.3535534",
                "0.1464466",
                "0.8535534",
                "omni_front",
                "top_right",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "-0.15",
                "0.15",
                "-0.05",
                "-0.3535534",
                "-0.3535534",
                "0.1464466",
                "0.8535534",
                "omni_front",
                "bottom_left",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "-0.15",
                "-0.15",
                "-0.05",
                "0.3535534",
                "-0.3535534",
                "-0.1464466",
                "0.8535534",
                "omni_front",
                "bottom_right",
            ],
        ),
    ]

    # Conditionally add static tfs for dex_board_green and dex_board_blue
    green_blue_found = False
    if os.path.exists(yaml_path):
        with open(yaml_path, "r") as f:
            try:
                poses = yaml.safe_load(f)
                for color in ["dex_board_green", "dex_board_blue"]:
                    if poses and color in poses:
                        pose = poses[color]
                        t = pose["translation"]
                        r = pose["rotation"]
                        dex_static_nodes.append(
                            Node(
                                package="tf2_ros",
                                executable="static_transform_publisher",
                                output="screen",
                                parameters=[{"use_sim_time": True}],
                                arguments=[
                                    str(t["x"]),
                                    str(t["y"]),
                                    str(t["z"]),
                                    str(r["x"]),
                                    str(r["y"]),
                                    str(r["z"]),
                                    str(r["w"]),
                                    pose["frame_id"],
                                    pose["child_frame_id"],
                                ],
                            )
                        )
                if len(dex_static_nodes) == 2:
                    green_blue_found = True
            except Exception as e:
                print(f"Error reading {yaml_path}: {e}")

    if green_blue_found:
        return LaunchDescription(static_nodes + dex_static_nodes)
    else:
        detect_node = Node(
            package="webots_spot",
            executable="detect_dex_board",
            output="screen",
            parameters=[{"use_sim_time": True}],
        )
        return LaunchDescription(static_nodes + [detect_node])
