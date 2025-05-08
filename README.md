# ROS2 Humble A* Turtlebot Navigation

This repository contains configuration and instructions for implementing A* pathfinding algorithm with TurtleBot3 in ROS2 Humble. It provides a complete setup for navigation and SLAM with the TurtleBot3 Waffle model using the A* algorithm for path planning.

## Overview

The project focuses on:
1. Setting up ROS2 Humble with TurtleBot3 packages
2. Creating maps with Cartographer SLAM
3. Configuring and using A* path planning via SmacPlanner2D
4. Navigating autonomously in Gazebo simulation environments

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble

### Installation

See the [Installation Guide](installation.md) for detailed setup instructions.

### Usage

1. **Create a Map**:
   ```bash
   # Terminal 1: Launch Gazebo simulation
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   
   # Terminal 2: Launch SLAM
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
   
   # Terminal 3: Control the robot
   ros2 run turtlebot3_teleop teleop_keyboard
   
   # Terminal 4: Save the map when done
   ros2 run nav2_map_server map_saver_cli -f ~/map
   ```

2. **Navigate with A* Path Planning**:
   ```bash
   # Terminal 1: Launch Gazebo simulation
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   
   # Terminal 2: Launch navigation with A* path planning
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
   ```

3. In RViz:
   - Set the initial pose using the "2D Pose Estimate" button
   - Set a navigation goal using the "Navigation2 Goal" button

For more detailed instructions, see the [Navigation Guide](navigation.md).

## A* Path Planning Configuration

This repository uses the A* algorithm for path planning via the SmacPlanner2D plugin. The configuration is set in the `waffle.yaml` parameter files to use the A* algorithm for efficient pathfinding.

See [Navigation Guide](navigation.md) for details on how to configure and verify the A* path planning setup.

## References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Navigation2 Documentation](https://navigation.ros.org/)

## License

This project is licensed under the MIT License - see the LICENSE file for details.
