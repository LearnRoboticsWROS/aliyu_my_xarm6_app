# 🦾 XARM6 + GAZEBO + MoveIt 2 Application packagge

This package is the application layer where we are going to run pick and place

---

## ⚙️ Prerequisites

clone in ~/aliyu_ws/src this repository


## Build and run

- each terminal that you about to open, make sure to source the environment
```bash
cd ~/aliyu_ws
colcon build
source install/setup.bash
```

- Terminal 1:
```bash
ros2 launch my_xarm6 spawn_xarm6_gripper_moveit_world.launch.py
```

- Terminal 2:
```bash
ros2 run my_xarm6_app pick_place
```

camera application will follow...
