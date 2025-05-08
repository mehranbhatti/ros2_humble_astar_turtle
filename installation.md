# Installation Guide for ROS2 Humble A* TurtleBot

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble

## Setup Instructions

### 1. Install Required Packages

```bash
# Install Gazebo packages for ROS2 Humble
sudo apt install ros-humble-gazebo-*

# Install Cartographer for SLAM
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros

# Install colcon build tools
sudo apt install python3-colcon-common-extensions
```

### 2. Create and Configure Workspace

```bash
# Create workspace directories
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/

# Clone required repositories
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ros-planning/navigation2.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Build the workspace
cd ~/turtlebot3_ws
colcon build --symlink-install
```

### 3. Set Environment Variables

Add the following to your `~/.bashrc` file:

```bash
# Source workspace
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc

# Set ROS domain ID
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc

# Source Gazebo setup
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

# Source ROS2 setup
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# Set TurtleBot3 model
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
```

Apply the changes:

```bash
source ~/.bashrc
```

## Verification

To verify your installation:

```bash
# Launch Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

You should see the Gazebo environment with a TurtleBot3 Waffle model.

## Troubleshooting

### Common Issues

1. **Package not found errors**
   - Make sure you've sourced your workspace: `source ~/turtlebot3_ws/install/setup.bash`
   - Try rebuilding the workspace: `cd ~/turtlebot3_ws && colcon build --symlink-install`

2. **Gazebo not launching**
   - Check if Gazebo is installed: `apt list --installed | grep gazebo`
   - Make sure your graphics drivers are up to date

3. **TurtleBot3 model errors**
   - Ensure the TURTLEBOT3_MODEL environment variable is set: `echo $TURTLEBOT3_MODEL`
   - Make sure all TurtleBot3 packages are properly installed
