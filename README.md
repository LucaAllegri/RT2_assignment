# ASSIGNMENT - RESEARCH TRACK 2

This project implements a ROS2 navigation system for a mobile robot in a 3D simulation environment (Gazebo/RViz).
The system allows the user to set a target pose (x, y, theta) for the robot, which then autonomously navigates
to the goal using a complete navigation based on TF2 frames.
Both the action client and the action server run as composable node plugins
within the same component container.
<br>
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
**The header files** - `include/robot_controller/` <br>
`action_client.hpp` and `action_server.hpp` declare the two classes `RobotActionClient` and `RobotActionServer` with all class name, methods and members. This separation allows manual_composition.cpp to instantiate both nodes knowing only their interface, without including the full implementation. <br>

**Source Files** - `src/`
  - **1) `action_client.cpp` - `RobotActionClient`**<br>
    It is the node that manages user input and sends navigation goals. This node:
    - creates the action client connected to the target action server;
    - in `send_goal()`, publishes a static TF transform from `odom` → `goal_frame` with the requested (x, y, theta) making the target visible in RViz;
    - sends the goal asynchronously
    - `goal_response_callback` — logs whether the goal was accepted or rejected by the server;
    - `feedback_callback` — logs the current relative position and distance received during navigation;
    - `result_callback` — handles the final result and updates the internal `goal_state_` string (`"SUCCEEDED"`, `"CANCELLED"`, `"ABORTED"`), which is polled by `manual_composition`;<br>
    - exposes `get_goal_state()`, `reset_goal_state()` and `cancel_current_goal()` to allow `manual_composition` to read the goal outcome and request cancellation directly.<br>
      
  - **2) `action_server.cpp` - `RobotActionServer`**<br>
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
      
  - **3) `manual_composition.cpp` — User Interface**<br>
    This is the entry point of the `manual_composition` executable. It provides an interactive loop that allows the user to send multiple goals sequentially and cancel them at any time:
    - asks the user for x, y, theta target values;
    - calls `talker->send_goal(x, y, th)` directly (no intermediate topics);
    - spawns a background thread that monitors stdin via `select()`: if the user types 'c' + ENTER, it sets a `cancel_requested` flag and `talker->cancel_current_goal()` is called from the main loop;
    - a pipe is used to unblock the background thread when the goal terminates naturally (without cancellation), avoiding it from remaining stuck on `select()`;
    - polls `talker->get_goal_state()` inside the main loop, exiting when the state becomes `"SUCCEEDED"`, `"ABORTED"` or `"CANCELLED"`;<br>
   
**Launch File** - `launch/controller.launch.py`<br>
Everything is started by a single launch file that brings up three elements simultaneously:<br>
<br>
**1) Simulation Environment $\rightarrow$** includes the `bme_gazebo_sensors` package launch, which is responsible for:
- running the Gazebo server and client with the 3D world
- providing the URDF robot description and sensors
- publishing odometry on `/odom`

**2) Component Container $\rightarrow$** both `RobotActionClient` and `RobotActionServer` are compiled as shared libraries and loaded as `ComposableNode` plugins into the same `component_container`, sharing the same process. The frame names (`world_frame_name`, `moved_frame_name`, `target_frame_name`) are passed as parameters to both nodes. The components, client + server, run in a separate xterm terminal (the "Feedback" window). This separates user interaction from logs and feedback.

<div align="center">
  <img src="/image_readme/feedback.png" alt="Feedback" width="500"/>
</div>
<br>

**3) User Interface $\rightarrow$** manual_composition is launched as a standalone node in a dedicated xterm terminal (the "UI" window), where the user can type the target x, y, theta values interactively, 'c' to canecl a goal or 'q' to quit.<br>

<div align="center">
  <img src="/image_readme/user_input.png" alt="User Input" width="500"/>
</div>
<br>

<br>🔴**TF2 FRAME STRUCTURE**<br>
The navigation relies on three TF frames:
| Frame | Publisher | Description |
|---|---|---|
| `odom` | fixed (world) | World reference frame |
| `base_link` | `RobotActionServer` (from `/odom`) | Robot's current position |
| `goal_frame` | `RobotActionClient` (static) | Target position set by the user |

<div align="center">
  <img src="/image_readme/frame_tree.png" alt="Frame Tree" width="650"/>
</div>

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
- an **xterm "UI"** terminal window where the user can insert the target pose (x, y, theta) and cancel goals
- an **xterm "Feedback"** terminal window showing navigation logs and action feedback

Before inserting the target goal in the user interface window, it is suggested on RVIZ window:
- display $\rightarrow$ add $\rightarrow$ TF $\rightarrow$ ok
- fixed frame $\rightarrow$ odom

In this way it is possible to see the robot's motion from the point of view of the world (odom).
