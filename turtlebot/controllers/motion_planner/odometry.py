# odometry module - dead reckoning pose estimation using wheel encoders

import math
from motion_planner.config import WHEEL_RADIUS, WHEEL_SEPARATION


class Odometry:
    # tracks robot pose (x, y, theta) from wheel encoder readings
    # uses differential drive kinematics

    def __init__(self):
        # pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # previous encoder values in radians
        self.prev_left_position = 0.0
        self.prev_right_position = 0.0

        # robot params
        self.wheel_radius = WHEEL_RADIUS
        self.wheel_separation = WHEEL_SEPARATION

        print("[Odometry] Initialized at (0.0, 0.0, 0.0)")

    def update(self, left_position, right_position):
        # update pose based on wheel encoder deltas
        # left_position, right_position in radians

        # calculate wheel distances traveled since last update
        delta_left = (left_position - self.prev_left_position) * self.wheel_radius
        delta_right = (right_position - self.prev_right_position) * self.wheel_radius

        # calculate robot center displacement and rotation
        delta_center = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_separation

        # update pose using mid-point approximation for better accuracy
        # use theta + delta_theta/2 for orientation during motion
        self.x += delta_center * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # store current encoder values for next update
        self.prev_left_position = left_position
        self.prev_right_position = right_position

    def get_pose(self):
        # returns current pose estimate (x, y, theta)
        # x, y in meters, theta in radians
        return (self.x, self.y, self.theta)

    def reset_pose(self, x=0.0, y=0.0, theta=0.0):
        # reset pose to specified values
        self.x = x
        self.y = y
        self.theta = theta
        self.prev_left_position = 0.0
        self.prev_right_position = 0.0
        print(f"[Odometry] Pose reset to ({x:.2f}, {y:.2f}, {theta:.2f})")
