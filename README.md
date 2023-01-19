# Robotics Arm Controller Application by OpenRR

## Overview

This project is to use a robotics arm using OpenRR without openrr-apps.

## Usage

```bash
cargo run --bin [uviz_arm / ros_arm / ros2_arm / arm]
```

## Example (Real robot, Ufactory Lite6)

### ROS

Launch ROS

```bash
roslaunch lite6_moveit_config realMove_exec.launch robot_ip:=192.168.x.xxx
```

Build and run the rust project

```bash
cargo run --bin ros_arm --features ros  -- --urdf-path "lite6.urdf" --end-link-name "joint_eef"
```

### ROS2

Launch ROS2

```bash
ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.x.xxx dof:=6 robot_type:=lite
```

Build and run the rust project

```bash
cargo run --bin ros2_arm --features ros2 -- --urdf-path "lite6.urdf" --end-link-name "joint_eef"
```

### Real robot

![GIF](https://j.gifs.com/mqj30r.gif)
