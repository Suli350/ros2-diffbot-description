# ROS2 DiffBot Description

Model a differential-drive robot in **URDF/xacro**, publish its TF tree and drive it
around in **RViz** with a lightweight kinematic simulator (no Gazebo needed).

![stack](https://img.shields.io/badge/ROS2-Humble-blue) ![python](https://img.shields.io/badge/python-3.10-green)

## What you learn

- Writing a robot in xacro: properties, macros, inertia formulas, materials
- `robot_state_publisher` and how `joint_states` turn into TF
- The `odom -> base_footprint -> base_link -> wheels/lidar/imu` frame tree
- Differential drive kinematics: twist <-> wheel speeds, saturation, exact integration
- Publishing `nav_msgs/Odometry` and broadcasting TF with `tf2_ros`
- Launch arguments and conditions

## Build

```bash
cd ~/ros2_ws/src && git clone https://github.com/<you>/ros2-diffbot-description.git
cd ~/ros2_ws && rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install && source install/setup.bash
```

## Run

Inspect the model with joint sliders:

```bash
ros2 launch diffbot_description display.launch.py
```

Drive it:

```bash
ros2 launch diffbot_description sim.launch.py              # then, in another terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 launch diffbot_description sim.launch.py demo:=true   # automatic figure eight
```

Useful inspection commands:

```bash
ros2 run tf2_tools view_frames          # writes frames.pdf
ros2 run tf2_ros tf2_echo odom base_link
xacro src/ros2-diffbot-description/diffbot_description/urdf/diffbot.urdf.xacro > /tmp/diffbot.urdf
check_urdf /tmp/diffbot.urdf
```

## Exercises

1. Add a camera link on a small mast and show it in RViz.
2. Add Gazebo plugins (`gazebo_ros_diff_drive`) and spawn the robot in Gazebo.
3. Add wheel encoder noise to the simulator and watch odometry drift.
