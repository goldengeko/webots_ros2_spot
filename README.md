# Webots ROS2 Spot

[![ROS2 Humble](https://github.com/MASKOR/webots_ros2_spot/actions/workflows/test_ros2_humble.yml/badge.svg?branch=main)](https://github.com/MASKOR/webots_ros2_spot/actions/workflows/test_ros2_humble.yml)

This is a ROS 2 package to simulate the Boston Dynamics spot in [webots](https://cyberbotics.com/) with Kinova Gen3 manipulator. Spot is able to walk around, to sit, standup and lie down. We also attached some sensors on spot, like a kinect and a 3D laser.

The world contains apriltags, a red line to test lane follower and objects for manipulation tasks.

## At this point you might have switched to this branch

Follow these steps

1. Switch to 2023 controller 
    ```
    cd src/webots_ros2
    git checkout 2023.1.2
    cd back/to/ros2_ws
    chmod +x src/webots_ros2/webots_ros2_driver/webots_ros2_driver/ros2_supervisor.py
    ```

2. Build packages and source the workspace
    ```
    colcon build --symlink-install
    source install/setup.bash
    ```

3. Add Kortex description and robotiq description and gripper controller
    ```
    sudo apt install ros-humble-kortex-description ros-humble-robotiq-description ros-humble-gripper-controllers
    ```

4. Add missing Meshes

    copy the 2f_140 folder in "opt/ros/humble/share/robotiq_description/meshes/visual" and "opt/ros/humble/share/robotiq_description/meshes/collision"

## Start
Starting the simulation:
```
ros2 launch webots_spot spot_launch.py
```

Send joint positions
```
ros2 topic pub --once /kinova_joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
  points: [
    {
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      accelerations: [],
      effort: [],
      time_from_start: { sec: 3, nanosec: 0 }
    }
  ]
}"
```

Motion Planning with MoveIt
```
ros2 launch webots_spot moveit_launch.py 
```
