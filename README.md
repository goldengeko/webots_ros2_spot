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

    Copy the 2f_140 folder in ```opt/ros/humble/share/robotiq_description/meshes/visual``` and ```opt/ros/humble/share/robotiq_description/meshes/collision```

5. If the IK solver is missing, install from [here](https://github.com/PickNikRobotics/pick_ik)

## Start
Starting the simulation:
```
ros2 launch webots_spot spot_launch.py
```


Motion Planning with MoveIt (please source install [moveit2](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html#create-a-colcon-workspace-and-download-tutorials))
```
ros2 launch webots_spot moveit_launch.py 
```

Realtime servoing (previous commands need to be launched and running; all commands in seperate tabs (duh!) )
```
ros2 launch webots_spot servo_launch.py

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
```
Note: The commands can be published on topics ```sevo_node/delta_twist_cmds``` and ```sevo_node/delta_joint_cmds```. Play around with the `moveit_servo_config.yaml` if servo launch throws errors

For servoing using keyboard(teleop) (acting a bit weird, but works)
```
ros2 run webots_spot teleop_servo 
```
