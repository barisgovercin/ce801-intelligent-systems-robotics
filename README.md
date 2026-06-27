# Mobile Robot Control with Fuzzy Logic and PID

**Module:** CE801 — Intelligent Systems and Robotics
**Student:** Muhammed Baris Govercin
**Registration No:** 2501385

## Overview

ROS 2 (`rclpy`) controllers for a differential-drive robot navigating with a
2D laser scanner. The robot performs **right-wall / edge following** and
**obstacle avoidance**, implemented three ways so the approaches can be
compared directly:

- a **fuzzy** edge follower vs. a **PID** edge follower (classical baseline)
- a **fuzzy** obstacle-avoidance controller
- a **combined architecture** that fuses obstacle avoidance with edge
  following via a fuzzy blender

Each node subscribes to `/scan` (`sensor_msgs/LaserScan`), groups the beams
into regions (front / left / right), and publishes velocity commands to
`/cmd_vel` (`geometry_msgs/Twist`) on a 0.2 s timer.

## Files

| File | Description |
|------|-------------|
| `Fuzzy_Right_Edge_Following.py` | Fuzzy controller keeping an ideal distance from the right wall |
| `PID_Right_Edge_Following.py` | Classical PID right-edge follower (baseline) |
| `Fuzzy_Obstacle_Avoidance.py` | Pure fuzzy obstacle avoidance |
| `Final_Architecture.py` | Combined: fuzzy obstacle avoidance fused with fuzzy edge following |
| `Robotics Presentation.pptx` | Project presentation |

## Fuzzy controller design

**Membership functions** (distance in metres):

| Set | Shape | Active region |
|-----|-------|---------------|
| `close_right` | falling | ≤ 0.25 m → 1.0, fades out by 0.45 m |
| `okay_right` | triangular | peak ≈ 0.45 m (between 0.25–0.65 m) |
| `far_right` | rising | from 0.5 m, saturates at 1.0 by 1.0 m |
| `near_front` | falling | ≤ 0.3 m → 1.0 |
| `far_front` | rising | from 0.5 m, saturates at 1.0 by 1.0 m |

**Edge-following rules** (fired strength → centroid defuzzification):

| Rule | Condition | Angular centroid | Linear centroid |
|------|-----------|------------------|-----------------|
| R1 | front near | +0.8 (hard left) | 0.08 (slow) |
| R2 | front far ∧ right close | +0.5 (soft left) | 0.18 |
| R3 | front near ∧ right far | −0.5 (soft right) | 0.18 |
| R4 | front far ∧ right okay | 0.0 (straight) | 0.30 (fast) |

**Obstacle-avoidance rules** add a corner-escape rule (front near ∧ both
sides close → strong +1.0 turn, near-zero linear) and forbid right turns for
safety. Defuzzification everywhere uses the **weighted-average (centroid)**
method: `Σ(strengthᵢ · centroidᵢ) / Σ strengthᵢ`.

**Fuzzy blender** (combined architecture): the front distance decides how
control is shared. When the front is *near*, obstacle avoidance dominates;
when *far*, edge following dominates; in the *middle* both share control. A
hard safety override switches to obstacle-avoidance-only when `near_front`
membership exceeds 0.3, and forward motion stops entirely below 0.22 m.

## PID controller

Right-wall following as a classical control baseline:

```
kp = 0.88,  ki = 0.0,  kd = 0.33     # effectively PD control
desired wall distance = 0.40 m
linear velocity = 0.3 m/s  (0 when an obstacle is < 0.2 m ahead)
angular velocity = kp·e + ki·∫e + kd·Δe,   e = desired − measured
```

The integral gain is left at zero (a steady cross-track offset was better
handled by the proportional + derivative terms without integral wind-up).

## Running

Requires a ROS 2 environment with a robot publishing `LaserScan` data
(e.g. a TurtleBot/ROSbot in Gazebo):

```bash
python3 Final_Architecture.py     # or any single controller above
```
