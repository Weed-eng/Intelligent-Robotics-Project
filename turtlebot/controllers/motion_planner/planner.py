# motion planner core logic - handles robot movement and control

import math
from controller import Robot
from motion_planner.config import (
    TIME_STEP,
    MAX_WHEEL_VELOCITY,
    MAX_LINEAR_VELOCITY,
    MAX_ANGULAR_VELOCITY,
    WHEEL_RADIUS,
    WHEEL_SEPARATION,
    LEFT_MOTOR_NAME,
    RIGHT_MOTOR_NAME,
    TEST_FORWARD_DURATION,
    TEST_ROTATE_DURATION,
    TEST_STOP_DURATION,
    DEFAULT_FORWARD_SPEED,
    DEFAULT_ROTATE_SPEED,
    K_LINEAR,
    K_ANGULAR,
    GOAL_TOLERANCE,
)
from motion_planner.odometry import Odometry


class MotionPlannerController:
    # main controller for motion planning
    # handles robot init, motor control, movement primitives

    def __init__(self):
        # init robot and devices
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # get motors
        self.left_motor = self.robot.getDevice(LEFT_MOTOR_NAME)
        self.right_motor = self.robot.getDevice(RIGHT_MOTOR_NAME)

        # set velocity control mode (position = infinity)
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        # start stopped
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # get position sensors (attached to motors)
        self.left_sensor = self.left_motor.getPositionSensor()
        self.right_sensor = self.right_motor.getPositionSensor()

        # enable position sensors
        self.left_sensor.enable(self.timestep)
        self.right_sensor.enable(self.timestep)

        # init odometry
        self.odometry = Odometry()

        print("[MotionPlanner] Initialized successfully")
        print(f"[MotionPlanner] Timestep: {self.timestep}ms")

    def set_wheel_velocities(self, left_speed, right_speed):
        # set left and right wheel speeds in rad/s
        # clamps to max velocity limits
        left_speed = max(-MAX_WHEEL_VELOCITY, min(MAX_WHEEL_VELOCITY, left_speed))
        right_speed = max(-MAX_WHEEL_VELOCITY, min(MAX_WHEEL_VELOCITY, right_speed))

        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def move_forward(self, speed=DEFAULT_FORWARD_SPEED):
        # move forward at given speed (rad/s)
        self.set_wheel_velocities(speed, speed)

    def rotate_left(self, speed=DEFAULT_ROTATE_SPEED):
        # rotate counterclockwise (left turn)
        self.set_wheel_velocities(-speed, speed)

    def rotate_right(self, speed=DEFAULT_ROTATE_SPEED):
        # rotate clockwise (right turn)
        self.set_wheel_velocities(speed, -speed)

    def stop(self):
        # stop robot
        self.set_wheel_velocities(0.0, 0.0)

    def normalize_angle(self, angle):
        # wrap angle to [-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))

    def distance_to_goal(self, goal):
        # calculate euclidean distance to goal
        # goal is (x, y) tuple
        x, y, _ = self.odometry.get_pose()
        dx = goal[0] - x
        dy = goal[1] - y
        return math.sqrt(dx*dx + dy*dy)

    def angle_to_goal(self, goal):
        # calculate desired heading to reach goal
        # returns angle in radians
        x, y, _ = self.odometry.get_pose()
        return math.atan2(goal[1] - y, goal[0] - x)

    def compute_velocity(self, goal):
        # p-controller: calculate velocity commands to reach goal
        # goal is (x, y) tuple
        # returns (v, omega) in m/s and rad/s

        # get current pose
        x, y, theta = self.odometry.get_pose()

        # calculate distance and heading error
        distance = self.distance_to_goal(goal)
        desired_heading = self.angle_to_goal(goal)
        heading_error = self.normalize_angle(desired_heading - theta)

        # check if goal reached
        if distance < GOAL_TOLERANCE:
            return (0.0, 0.0)

        # proportional control
        v = K_LINEAR * distance
        omega = K_ANGULAR * heading_error

        # saturate to max velocities
        v = max(0.0, min(MAX_LINEAR_VELOCITY, v))
        omega = max(-MAX_ANGULAR_VELOCITY, min(MAX_ANGULAR_VELOCITY, omega))

        return (v, omega)

    def set_velocity_commands(self, v, omega):
        # convert linear and angular velocities to wheel velocities
        # v in m/s, omega in rad/s
        # uses differential drive inverse kinematics

        # wheel velocities in m/s
        v_left = v - (omega * WHEEL_SEPARATION / 2.0)
        v_right = v + (omega * WHEEL_SEPARATION / 2.0)

        # convert to rad/s (angular velocity of wheels)
        omega_left = v_left / WHEEL_RADIUS
        omega_right = v_right / WHEEL_RADIUS

        self.set_wheel_velocities(omega_left, omega_right)

    def run_test_sequence(self):
        # runs 6-phase test loop to verify motor control and odometry
        # forward -> rotate left -> stop -> forward -> rotate right -> stop -> repeat
        print("\n[TEST] Starting movement test sequence...")
        print("[TEST] Use Pause/Stop button in Webots to stop")

        step_count = 0
        test_phase = 0
        pose_print_counter = 0

        # convert durations to timesteps
        FORWARD_STEPS = int(TEST_FORWARD_DURATION * 1000 / TIME_STEP)
        ROTATE_STEPS = int(TEST_ROTATE_DURATION * 1000 / TIME_STEP)
        STOP_STEPS = int(TEST_STOP_DURATION * 1000 / TIME_STEP)
        POSE_PRINT_INTERVAL = int(500 / TIME_STEP)  # print pose every 0.5s

        while self.robot.step(self.timestep) != -1:
            step_count += 1
            pose_print_counter += 1

            # update odometry
            left_pos = self.left_sensor.getValue()
            right_pos = self.right_sensor.getValue()
            self.odometry.update(left_pos, right_pos)

            # print pose periodically
            if pose_print_counter >= POSE_PRINT_INTERVAL:
                x, y, theta = self.odometry.get_pose()
                print(f"[POSE] x={x:6.3f}m  y={y:6.3f}m  θ={theta:6.3f}rad")
                pose_print_counter = 0

            if test_phase == 0:
                if step_count == 1:
                    print("[TEST] Phase 0: Moving forward...")
                    self.move_forward()
                elif step_count > FORWARD_STEPS:
                    test_phase = 1
                    step_count = 0

            elif test_phase == 1:
                if step_count == 1:
                    print("[TEST] Phase 1: Rotating left...")
                    self.rotate_left()
                elif step_count > ROTATE_STEPS:
                    test_phase = 2
                    step_count = 0

            elif test_phase == 2:
                if step_count == 1:
                    print("[TEST] Phase 2: Stopping...")
                    self.stop()
                elif step_count > STOP_STEPS:
                    test_phase = 3
                    step_count = 0

            elif test_phase == 3:
                if step_count == 1:
                    print("[TEST] Phase 3: Moving forward again...")
                    self.move_forward()
                elif step_count > FORWARD_STEPS:
                    test_phase = 4
                    step_count = 0

            elif test_phase == 4:
                if step_count == 1:
                    print("[TEST] Phase 4: Rotating right...")
                    self.rotate_right()
                elif step_count > ROTATE_STEPS:
                    test_phase = 5
                    step_count = 0

            elif test_phase == 5:
                if step_count == 1:
                    print("[TEST] Phase 5: Final stop")
                    print("[TEST] Test sequence complete! Restarting...\n")
                    self.stop()
                elif step_count > STOP_STEPS:
                    test_phase = 0
                    step_count = 0

    def run_goal_seeking_test(self, waypoints):
        # test goal-seeking p-controller with multiple waypoints
        # waypoints is list of (x, y) tuples
        print("\n[GOAL] Starting goal-seeking test...")
        print(f"[GOAL] Waypoints: {waypoints}")
        print("[GOAL] Use Pause/Stop button in Webots to stop")

        current_waypoint_idx = 0
        pose_print_counter = 0
        POSE_PRINT_INTERVAL = int(500 / TIME_STEP)  # print every 0.5s

        while self.robot.step(self.timestep) != -1:
            pose_print_counter += 1

            # update odometry
            left_pos = self.left_sensor.getValue()
            right_pos = self.right_sensor.getValue()
            self.odometry.update(left_pos, right_pos)

            # check if all waypoints reached
            if current_waypoint_idx >= len(waypoints):
                print("[GOAL] All waypoints reached! Stopping...")
                self.stop()
                break

            # get current goal
            goal = waypoints[current_waypoint_idx]

            # compute velocity commands using p-controller
            v, omega = self.compute_velocity(goal)

            # apply commands
            self.set_velocity_commands(v, omega)

            # check if goal reached
            distance = self.distance_to_goal(goal)
            if distance < GOAL_TOLERANCE:
                print(f"[GOAL] Reached waypoint {current_waypoint_idx}: {goal}")
                current_waypoint_idx += 1
                if current_waypoint_idx < len(waypoints):
                    next_goal = waypoints[current_waypoint_idx]
                    print(f"[GOAL] Moving to next waypoint: {next_goal}")

            # print pose and progress periodically
            if pose_print_counter >= POSE_PRINT_INTERVAL:
                x, y, theta = self.odometry.get_pose()
                print(f"[POSE] x={x:6.3f}m y={y:6.3f}m θ={theta:6.3f}rad | goal={goal} dist={distance:5.3f}m")
                pose_print_counter = 0
