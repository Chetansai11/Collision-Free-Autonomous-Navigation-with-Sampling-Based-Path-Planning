# ROS 2 Navigation System with A* and RRT* Planning

This project implements a **complete end-to-end mobile robot navigation pipeline** in ROS 2, including **simulation, global path planning, and path tracking control**, with rich RViz visualization. The system supports both **grid-based A\*** and **sampling-based RRT\*** planners and demonstrates safe, collision-free navigation in environments with known obstacles.

The focus of this project is on **algorithmic reasoning, system integration, and motion planning fundamentals**, rather than relying on existing ROS navigation stacks.

![RRT* Navigation](images/projectz11.gif)

---

## 🚀 System Overview

The system is composed of **three main ROS 2 nodes**, each responsible for a core navigation function:

| Simulator | -----> | Global Planner | -----> | Controller |
| (sim1) | ----->| pose | (A* / RRT*) |-----> | path |----->| (Pure Pursuit) |



### Key Capabilities
- Continuous-space motion planning using **RRT\*** with rewiring
- Grid-based **A\*** planner for baseline comparison
- Obstacle inflation using robot radius for safety-aware planning
- Real-time trajectory tracking using a **lookahead-based controller**
- RViz visualization of **planned vs executed paths** and safety buffers

---

## 🧠 Node Architecture

### 1. Simulator Node (`sim1`)
- Publishes the robot’s ground-truth pose as `geometry_msgs/Pose2D`
- Topic:
  - `/pose`

This node emulates the robot state and serves as the feedback source for both planning and control.

---

### 2. Global Planner Node (`path_planner`)
The planner computes a **collision-free global path** from the robot’s current pose to a user-defined goal.

#### Subscriptions
- `/pose` (`Pose2D`) — current robot position
- `/goal_pose` (`PoseStamped`) — goal sent from RViz (2D Nav Goal)

#### Publications
- `/planned_path` (`nav_msgs/Path`) — global path to be tracked

#### Supported Planning Algorithms

##### 🔹 A* (Grid-Based)
- Operates on a precomputed occupancy grid
- Obstacles are inflated by the robot radius
- Used primarily as a **baseline reference**
- Fast but limited to discretized environments

##### 🔹 RRT* (Sampling-Based, Continuous Space)
Primary focus of this project.

**Algorithm Details:**
- Random sampling in continuous workspace  
  \([min_x, max_x] × [min_y, max_y]\)
- Goal-biased sampling to accelerate convergence
- Fixed step-size extension toward nearest tree node
- Collision checking against **inflated circular obstacles**
- Cost-aware parent selection
- **Rewiring** nearby nodes to improve path optimality
- Path reconstruction via parent backtracking once within goal radius

RRT* naturally handles:
- Continuous state spaces  
- Irregular obstacle layouts  
- Progressive path improvement  

---

### 3. Controller Node (`controller`)
The controller tracks the planned path and generates velocity commands.

#### Subscriptions
- `/planned_path` (`nav_msgs/Path`)
- `/pose` (`Pose2D`)

#### Publications
- `/cmd_vel` (`geometry_msgs/Twist`)
- `/executed_path` (`nav_msgs/Path`)

#### Control Strategy: Pure Pursuit / Lookahead Control
- Selects a waypoint ahead of the robot at a configurable lookahead distance
- Computes heading error to the target point
- Uses proportional control:
  - `linear_k`, `angular_k`
- Applies velocity saturation:
  - `max_lin`, `max_ang`
- Rotates in place if heading error is large
- Stops when the robot reaches goal tolerance

This design ensures:
- Smooth trajectory tracking
- Stable behavior at higher speeds
- Clear separation between planning and control layers

---

## 🖼️ Visualization (RViz)

The system includes extensive visualization to improve interpretability.

### Obstacles
Each obstacle is visualized using **two marker layers**:
- **Solid red cylinder** — actual physical obstacle
- **Larger translucent cylinder** — inflated obstacle  
  *(robot radius = 0.5 + safety margin = 0.2)*

This makes the planner’s safety assumptions explicit and helps explain:
- Why paths bend early
- Why some goals are unreachable

### Paths
- **Planned Path** (`/planned_path`) — output of the planner
- **Executed Path** (`/executed_path`) — actual robot trajectory

---

## 📊 Results and Observations

### What Worked Well
- RRT* consistently found **valid, collision-free paths** in cluttered environments
- Rewiring visibly shortened and smoothed paths as iterations increased
- Goals inside inflated obstacles were correctly rejected
- Straight-line goals resulted in near-optimal paths
- Controller followed paths stably once gains and lookahead were tuned

### Limitations
- In highly cluttered environments, RRT* paths can appear jagged without:
  - More iterations
  - Post-processing path smoothing  
This behavior is expected for sampling-based planners.

Overall, the results matched expectations and clearly demonstrate **safe and reasonable autonomous navigation behavior**.

---

## 🧪 How to Run the System

### 1. Build the Workspace
```bash
cd ~/ros2_workspace
colcon build
source install/setup.bash
ros2 launch projectz nav_launch.py
```

This launch file starts:
sim1 (simulator)
path_planner
controller
rviz2 with nav.rviz
