# ASSIGNMENT - RESEARCH TRACK 2

This project implements a ROS2 navigation system for a mobile robot in a 3D simulation environment (Gazebo/RViz).
The system allows the user to set a target pose (x, y, theta) for the robot, which then autonomously navigates
to the goal using a complete navigation based on TF2 frames.
Both the action client and the action server run as composable node plugins
within the same component container.

<div align="center">
  <img src="/image_readme/RobotMotion.gif" alt="Robot motion" width="500"/>
</div>

<br>

<br>🔴**CUSTOM COMMUNICATION INTERFACES**<br>
The action custom comunication `Target.action` used between the client and the server is in the dedicated package `action_interfaces`, composed as follows:
- **Goal**: `float64[3] goal_pose` — the desired target as [x, y, theta]
- **Feedback**: `float64[3] current_pose` — current relative position and distance [x, y, distance]
- **Result**: `float64[3] final_pose` — the final pose reached and final distance [x, y, distance]

<br>🔴**ARCHITECTURE AND NODES**<br>
The `robot_controller` package is structured as follows:
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
**The header files** - `include/robot_controller/` 
`action_client.hpp` and `action_server.hpp` to declare the two classes `RobotActionClient` and `RobotActionServer` with all class name, methods and members. This separation allows manual_composition.cpp to instantiate both nodes knowing only their interface, without including the full implementation. <br>

**Source Files** - `src/`
  - **1) action_client.cpp - RobotActionClient**
    It is the node that manages user input and sends navigation goals. This node:
    - creates the action client connected to the target action server;
    - in `send_goal()`, publishes a static TF transform from `odom` → `goal_frame` with the requested x, y, theta, making the target visible in RViz;
    - sends the goal asynchronously
    - `goal_response_callback` — logs whether the goal was accepted or rejected by the server;
    - `feedback_callback` — logs the current relative position and distance received during navigation;
    - `result_callback` — handles the final result (IMAGE)
      
  - **2) action_server.cpp - RobotActionServer**
    It is the node that implements the full navigation logic of the robot. This node:
    - subscribes to `/odom` and in `odom_callback` broadcasts a dynamic TF transform from `odom` → `base_link`, keeping the robot's position continuously updated in the TF tree;
    - uses a TF2 listener + buffer to look up the transform from `base_link` → `goal_frame` at each control loop iteration, obtaining the relative error vector (x, y);
    - `handle_goal` — always accepts incoming goals and logs the requested target;
    - `handle_cancel` — accepts cancellation requests at any time;
    - `handle_accepted` — immediately spawns execute in a detached thread to avoid blocking the executor;
    - `execute`:
      - aligns the robot toward the goal;
      - moves forward proportionally to the remaining distance;
      - stops the robot when distance < 0.01 m;
      - checks for cancellation at every iteration, stopping the robot cleanly if requested.
  - **3) manual_composition.cpp — Standalone**
    This is the entry point of the `manual_composition` executable: instead of relying on ROS 2's dynamic composition system (which loads plugins via `component_container` at runtime), here the composition happens manually and statically in code.
    - Instantiates both nodes and adds them to the same executor — both RobotActionClient and RobotActionServer live in the same process under a SingleThreadedExecutor, exactly as they would inside a `component_container`
    - Collects user input before spinning
    - once `spin()` is called, the odometry callbacks update the TF tree, the server navigates toward the goal, and the client receives feedback and the final result.<br>
   
**Launch File** - `launch/controller.launch.py`
Everything is started by a single launch file that brings up three elements simultaneously:
**1) Simulation Environment $\rightarrow$** includes the `bme_gazebo_sensors` package launch, which is responsible for:
- running the Gazebo server and client with the 3D world
- providing the URDF robot description and sensors
- publishing odometry on `/odom`

**2) Component Container $\rightarrow$** both `RobotActionClient` and `RobotActionServer` are compiled as shared libraries and loaded as `ComposableNode` plugins into the same `component_container`, sharing the same process. The frame names (`world_frame_name`, `moved_frame_name`, `target_frame_name`) are passed as parameters to both nodes.

**3) User Interface $\rightarrow$** manual_composition is launched as a standalone node, opened in a dedicated terminal window where where the user can type the target x, y, theta values interactively. (IMAGE)

<br>🔴**TF2 FRAME STRUCTURE**<br>
The navigation relies on three TF frames:
| Frame | Publisher | Description |
|---|---|---|
| `odom` | fixed (world) | World reference frame |
| `base_link` | `RobotActionServer` (from `/odom`) | Robot's current position |
| `goal_frame` | `RobotActionClient` (static) | Target position set by the user |
 (IMAGE)

 The server looks up the transform `base_link → goal_frame` at each control loop iteration. When this transform's translation norm is below the threshold, the goal is declared reached.

 <br>🔴**HOW TO RUN THE SIMULATION**<br>
 First, build the workspace:
 ```bash
colcon build
source install/setup.bash
```
In a terminal, launch all nodes with:
 
```bash
ros2 launch robot_controller controller.launch.py
```
This command will open:
- the **Gazebo** simulation environment with the robot
- the **RViz** visualizer showing the TF frames and the robot
- an **xterm** terminal window where the user can insert the target pose (x, y, theta)

I suggest before insert the target goal in te user interface window, on RVIZ window
- display -> add -> TF -> ok
- fixed frame -> odom
