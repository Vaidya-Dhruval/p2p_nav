# p2p_nav

# P2P Navigation Action Server for Neobotix ROX ARGO

This repository provides a ROS 2 action server that enables point-to-point navigation for the Neobotix ROX ARGO robot. The action server integrates with the existing navigation stack to allow the robot to receive goals and navigate autonomously to the specified positions.

## Prerequisites

Before starting, ensure the following:

1. **ROS 2 Jazzy** is installed and properly configured.
2. **Neobotix ROX ARGO** robot setup and operational.
3. **Required ROS 2 Packages**:
    - `rox_bringup`
    - `rox_navigation`
    - `nav2_msgs`
    - `p2p_navigation_interfaces`
  

## Step-by-Step Guide

### Step 1: Set Up the Robot Bringup (Simulated or Real)

To bring up the robot and initialize all the components, including sensors, actuators, and communication, use the following launch command. This prepares the robot system for interaction.

```bash
ros2 launch rox_bringup bringup_sim_launch.py
```
Then start the navigation.

```bash
ros2 launch rox_navigation navigation.launch.py
```
If you just want to start the server 

```bash
ros2 launch p2p_navigation p2p_nav_server.launch.py
```
TO launch the full service (server+client)

```bash
ros2 launch p2p_navigation test_p2p_navigation.launch.py
``` 

it'll send the 2,3 goal as to the "/NavigateToPose" action.
