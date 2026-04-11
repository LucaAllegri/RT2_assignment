# ASSIGNMENT - RESEARCH TRACK 2

This project implements a ROS2 navigation system for a mobile robot in a 3D simulation environment (Gazebo/RViz).
The system allows the user to set a target pose (x, y, theta) for the robot, which then autonomously navigates
to the goal using a complete navigation based on TF2 frames.
Both the action client and the action server run as composable node plugins
within the same component container.

<br>

<br>🔴**CUSTOM COMMUNICATION INTERFACES**<br>
The action custom comunication `Target.action` used between the client and the server is in the dedicated package `action_interfaces` composed in this way:
- **Goal**: `float64[3] goal_pose` — the desired target as [x, y, theta]
- **Feedback**: `float64[3] current_pose` — current relative position and distance [x, y, distance]
- **Result**: `float64[3] final_pose` — the final pose reached and final distance [x, y, distance]

<br>🔴**ARCHITECTURE AND NODES**<br>
The `robot_controller`'s architecture package is formed as:
```bash
robot_controller/
├── CMakeLists.txt
├── include
│   └── robot_controller
│       ├── action_client.hpp
│       └── action_server.hpp
├── launch
│   └── controller.launch.py
├── package.xml
└── src
    ├── action_client.cpp
    ├── action_server.cpp
    └── manual_composition.cpp
```
We have the:
- **The header files** -> `action_client.hpp` and `action_server.hpp` to declare the two classes `RobotActionClient` and `RobotActionServer`
