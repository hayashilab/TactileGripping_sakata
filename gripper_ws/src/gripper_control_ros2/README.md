# gripper_control_ros2

ROS 2 (ament_python) package that ports the original ROS 1 gripper controller node to ROS 2.

## Build
```bash
cd <your_ros2_ws>/src
# copy this package folder here
cd ..
rosdep install --from-paths src -i -y
colcon build --symlink-install
. install/setup.bash
```

## Run
```bash
ros2 launch gripper_control_ros2 gripper_control.launch.py
```

## Topics / Service
- Subscribed:
  - `gripper/gripper_pose_in_step` (std_msgs/msg/Int32)
  - `gripper/gripper_pose_in_mm`   (std_msgs/msg/Int32)
- Service:
  - `gripper/set_zero` (std_srvs/srv/Empty)
