Kalman Prediction Module

This module takes LiDAR-based detections and maintains Kalman filters for each tracked human/object.

It outputs predicted:

future x, y position

velocity

direction

uncertainty

safety-radius-relevant metadata

These predictions are passed to the Adaptive DWA module.

Run

Used internally by main_controller.
No standalone Webots execution required
