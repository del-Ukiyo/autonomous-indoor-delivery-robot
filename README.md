# Autonomous Indoor Delivery Robot - Webots

3003ICT Programming for Robotics | Group NA12 | Griffith University

## Project Overview

An autonomous mobile robot simulated in Webots that navigates a hospital corridor,
avoids obstacles, and delivers to a red target using camera-based detection and
a four-state Finite State Machine.

## Group Members

| Name | Student ID |
|---|---|
| Kavin Imesh Kothalawala | s5317344 |
| Youssef El-Samman | s5454855 |

## Specialisation

Autonomous Robotics (Webots)

## How to Run

1. Open Webots
2. File - Open World - select worlds/autonomous-indoor-delivery-robot.wbt
3. The robot_controller will load automatically
4. Press Play to run the simulation

## Robot Behaviour

The robot uses a four-state FSM:

- WANDER: follows the right wall down the corridor
- AVOID: steers away from obstacles using IR sensors
- TARGET_SEEK: aligns to red target using camera centroid
- ARRIVAL: stops and activates LED on delivery complete

## Sensors Used

- 8x IR proximity sensors (ps0-ps7) for obstacle detection
- Camera (52x39px) for red target identification
- Wheel encoders for odometry position tracking
