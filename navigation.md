# Navigation Guide with A* Path Planning

## Overview

This guide explains how to configure and use A* path planning with TurtleBot3 in ROS2 Humble. The A* algorithm is implemented through the SmacPlanner2D plugin in the Navigation2 stack.

## Configure A* Path Planning

To use A* as the path planning algorithm, modify the configuration files:

### 1. Main Configuration

```bash
# Navigate to the parameter directory
cd ~/turtlebot3_ws/src/turtlebot/turtlebot3_navigation2/param

# Edit the configuration file
nano waffle.yaml
```

Replace the `planner_server` section with:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"  # Changed from NavfnPlanner to SmacPlanner2D
      tolerance: 0.5
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 2.0
      smooth_path: true
      downsample_costmap: false
      downsampling_factor: 1
      use_final_approach_orientation: false
      minimum_turning_radius: 0.4
```

### 2. Humble-specific Configuration

```bash
# Edit the Humble-specific configuration
cd ~/turtlebot3_ws/src/turtlebot/turtlebot3_navigation2/param/humble
nano waffle.yaml
```

Replace the `planner_server` section with the same configuration as above.

### 3. Verify A* Configuration

After launching navigation, verify the A* configuration:

```bash
ros2 param get /planner_server GridBased.use_astar
```

## Creating a Map

Before navigation, you need to create a map:

1. **Launch Gazebo Simulation**:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Start SLAM**:
   ```bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
   ```

3. **Control the TurtleBot**:
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

4. **Save the Map** (after exploring the environment):
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/map
   ```

## Navigating with A* Path Planning

1. **Launch Gazebo**:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Launch Navigation**:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
   ```

3. **RViz Configuration**:
   - Click the "2D Pose Estimate" button
   - Click on the map where the robot is and drag to set direction
   - Click the "Navigation2 Goal" button to set a destination

## Understanding A* in ROS2 Navigation

The A* algorithm in Navigation2 is implemented in the `nav2_smac_planner` package. The `a_star.cpp` file contains the core algorithm, and the `SmacPlanner2D` plugin (in `smac_planner2d.cpp`) uses this implementation.

Key points:

- A* finds the optimal path using heuristic search
- It uses an open set (frontier) and closed set (visited nodes)
- Each node is evaluated using f(n) = g(n) + h(n) where:
  - g(n) is the cost from start to current node
  - h(n) is the estimated cost from current node to goal

The SmacPlanner2D plugin is configured to use A* and provides parameters to tune the algorithm's behavior.

## Customizing A* Behavior

You can modify these parameters in the configuration file:

- `tolerance`: Goal tolerance in meters
- `allow_unknown`: Whether to allow planning through unknown space
- `max_iterations`: Maximum number of iterations
- `max_planning_time`: Maximum time allowed for planning in seconds
- `smooth_path`: Whether to smooth the path

For more advanced customization, you can create your own planning plugin using the A* implementation from `a_star.cpp`.
