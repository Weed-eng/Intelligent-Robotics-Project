# SLAM & Localization Module
Author: <Team Member Name>

This module performs localization for the robot using:
- wheel odometry
- LiDAR observations (optional)
- map correlation / occupancy grid updates (optional)

It outputs:
- robot pose (x, y, θ)
which is used by:
- DWA planner
- Kalman prediction (to transform detections)
- Adaptive safety logic

## Files
- `slam.c` or `slam.py`: main SLAM/localization logic
- `Makefile`: build instructions

## How to Run
This module is called internally by `main_controller`
and is not executed as a standalone Webots controller.

## Responsibilities
This controller is responsible for:
- maintaining the robot's global pose
- updating map/occupancy grid (if enabled)
- providing pose to the navigation stack
