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

## ⚙️ Vision application

- You need to create a custom service for structure the position of the object detected

```bash
cd ~/aliyu_ws/src
ros2 pkg create my_xarm6_interfaces --build-type ament_cmake
```
- the structure is very simple, we just need to use CmakeLists.txt for creating a srv
my_xarm6_interfaces/
 ├── CMakeLists.txt
 ├── package.xml
 └── src/

```bash
cd my_xarm6_interfaces
mkdir srv
nano srv/ObjectPosition.srv
```
in this file copy and paste the structure of the service:

```bash
string object_name
---
float64 x
float64 y
float64 z
bool success
```
Save ((CTRL+O, INVIO, CTRL+X)

- modify the CMakeLists.txt

```bash
cmake_minimum_required(VERSION 3.8)
project(my_xarm6_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Genera le interfacce
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ObjectPosition.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- modify the package.xml file:
```bash
<?xml version="1.0"?>
<package format="3">
  <name>my_xarm6_interfaces</name>
  <version>0.0.1</version>
  <description>Service definitions for my_xarm6_app</description>
  <maintainer email="ros.master.ai@gmail.com">fra</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- build
```bash
cd ~/aliyu_ws
colcon build --packages-select my_xarm6_interfaces
source install/setup.bash
```

-run the application with pick and place with the vision system that gets the center of the object (remember to source!)

- Terminal 1:
```bash
ros2 launch my_xarm6 spawn_xarm6_gripper_moveit_world.launch.py
```

- Terminal 2:
```bash
ros2 run my_xarm6_app object_position_server.py
```

- Terminal 3:
```bash
ros2 run my_xarm6_app pick_place_vision.py
```

You should see the robot that run a pick and place with the position given by the 3D camera


## ⚙️ Integration with LLM - Ollama

- We are going totackle this integration in 2 step

- Step 1: User prompt, Ollama reply by parsing a JSON file on the command to pass to the controller.
We will create a RO2 node that traslate JSON data in command to send to MoveIt

- Step 2: reasoning loop orchestrated from a node ROS2 using LLM as tool


## Step 1:

- install ollama:
```bash
curl -fsSL https://ollama.com/install.sh | sh
```

- check:
```bash
ollama --version
```
- Get the model
```bash
ollama pull llama3
```
python3 -m venv venv
source venv/bin/activate
pip install requests

- Test Ollama:

cd ~/my_xarm6_app 
python -m my_xarm6_app.llm.test_ollama

# Architecture Step 1
Terminal (ros2 topic pub)
        │
        ▼
/llm_command (String)
        │
        ▼
[llm_command_node] → call Ollama → generate PoseStamped → pub /target_pose
        │
        ▼
[move_to_position_from_llm] → call IK → FollowJointTrajectory
        │
        ▼
robot in Gazebo moves

- test
- Terminal 1:
```bash
ros2 launch my_xarm6 spawn_xarm6_gripper_moveit_world.launch.py
```

- Terminal 2:
```bash
ros2 run my_xarm6_app move_to_position_llm
```

- Terminal 3:
```bash
cd ~/my_xarm6_app
source venv/bin/activate
ros2 launch my_xarm6_app llm_command_node
```

- Terminal 4:
```bash
ros2 topic pub /llm_command std_msgs/String "{data: 'move the TCP to x 0.3 y 0.5 z 0.3 keeping current orientation'}"
```



