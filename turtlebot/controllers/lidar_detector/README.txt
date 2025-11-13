Purpose of This Module:

This controller performs raw LiDAR-based perception for the robot.
It extracts structured detection data from LiDAR and prepares it for:

-Kalman-based human motion prediction (Waleed’s module)
-Adaptive DWA safety logic
-Collision avoidance and human-aware navigation

This is the first stage of the perception pipeline.

What This Module Does:

-Reads LiDAR scan.
-Converts ranges → polar + Cartesian coordinates (x, y).
-Packages each valid LiDAR return as a detection point.
-Sends detection data to the next stage (Kalman Tracker).
-Performs clustering to group points into objects.

This module does not move the robot — it only senses.
