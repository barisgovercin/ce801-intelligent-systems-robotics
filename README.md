# Mobile Robot Control with Fuzzy Logic and PID

**Module:** CE801 — Intelligent Systems and Robotics
**Student:** Muhammed Baris Govercin
**Registration No:** 2501385

## Overview

ROS 2 (`rclpy`) controllers for a differential-drive mobile robot that
navigates using a 2D laser scanner (`LaserScan`). The robot performs
**wall / right-edge following** and **obstacle avoidance** using
Fuzzy Logic Controllers (FLC), compared against a classical **PID**
controller, and finally a combined architecture that fuses obstacle
avoidance with edge following.

## Files

| File | Description |
|------|-------------|
| `Fuzzy_Obstacle_Avoidance.py` | Pure fuzzy logic obstacle avoidance (front/left/right regions) |
| `Fuzzy_Right_Edge_Following.py` | Fuzzy controller that keeps the robot at an ideal distance from the right wall |
| `PID_Right_Edge_Following.py` | Classical PID right-edge follower (baseline for comparison) |
| `Final_Architecture.py` | Combined behaviour: fuzzy obstacle avoidance fused with fuzzy right-edge following |
| `Robotics Presentation.pptx` | Project presentation |

## How it works

Each controller subscribes to `/scan` (`sensor_msgs/LaserScan`), groups the
ranges into regions (front, front-left, front-right, left, right), evaluates
fuzzy membership functions over those distances, and publishes velocity
commands to `/cmd_vel` (`geometry_msgs/Twist`).

The fuzzy controllers define membership functions such as `close`, `okay`,
and `far` for each region and combine the activated rules to produce smooth
linear and angular velocity outputs.

## Running

Requires a ROS 2 environment with a simulated or real robot publishing
`LaserScan` data (e.g. TurtleBot in Gazebo).

```bash
python3 Final_Architecture.py
```
