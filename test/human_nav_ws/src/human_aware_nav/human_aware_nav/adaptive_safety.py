# adaptive safety module with ego-motion compensation and timer-based monitoring
# subscribes to /tracked_objects, /cmd_vel_raw, /odom
# publishes safe /cmd_vel continuously at 20Hz

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

import math
import time


class AdaptiveSafety(Node):
    def __init__(self):
        super().__init__('adaptive_safety')

        # subscribe to tracked objects from kalman tracker
        self.tracks_sub = self.create_subscription(
            MarkerArray,
            '/tracked_objects',
            self.tracked_objects_callback,
            10
        )

        # subscribe to raw velocity commands (from teleop or Nav2)
        self.cmd_raw_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_raw_callback,
            10
        )

        # subscribe to odometry for robot velocity (ego-motion compensation)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # publish filtered safe velocity commands
        self.cmd_safe_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # robot velocity from odometry (for ego-motion compensation)
        self.robot_vx = 0.0  # linear velocity in x (m/s)
        self.robot_wz = 0.0  # angular velocity around z (rad/s)

        # last raw command for timer-based re-evaluation
        self.last_raw_cmd = Twist()
        self.last_raw_cmd_time = 0.0
        self.cmd_timeout = 0.5  # stop if no command for this long

        # timer for continuous safety monitoring (20 Hz)
        self.safety_timer = self.create_timer(0.05, self.safety_timer_callback)

        # store positions and smoothed velocities of tracked objects
        # key: (ns, id) -> dict with x, y, t, vx_smooth, vy_smooth
        self.last_positions = {}

        # velocity smoothing (low-pass filter)
        self.velocity_alpha = 0.3  # higher = more responsive, lower = smoother

        # current threat state (from moving objects only)
        self.current_ttc = None
        self.current_dist = None
        self.last_threat_time = time.time()
        self.consecutive_threats = 0
        self.min_consecutive = 3  # require consecutive detections

        # safety thresholds (tuned for scaled pedestrians at ~0.5 m/s)
        self.approach_speed_threshold = 0.3  # m/s - object must approach faster than this
        self.ttc_stop_threshold = 1.5        # s - hard stop if TTC below this
        self.ttc_slow_threshold = 3.0        # s - start slowing if TTC below this
        self.emergency_stop_dist = 0.25      # m - hard stop if anything this close

        # distance filters
        self.min_detection_dist = 0.15  # ignore very close (noise)
        self.max_detection_dist = 3.0   # ignore far objects

        # sanity check - ignore unrealistic velocities
        self.max_reasonable_speed = 2.0  # m/s

        # threat timeout
        self.threat_timeout = 0.5  # s

        # closest object distance (for emergency stop)
        self.closest_object_dist = float('inf')

        self.get_logger().info(
            "Adaptive safety started (timer-based, ego-motion compensated, "
            f"approach_threshold={self.approach_speed_threshold} m/s)"
        )

    def odom_callback(self, msg: Odometry):
        # extract robot velocity from odometry
        self.robot_vx = msg.twist.twist.linear.x
        self.robot_wz = msg.twist.twist.angular.z

    def tracked_objects_callback(self, msg: MarkerArray):
        # process tracked objects, compute compensated velocities, detect threats
        now = time.time()

        candidate_ttc = None
        candidate_dist = None
        threat_found = False
        min_dist = float('inf')

        for m in msg.markers:
            # only process position markers (not velocity arrows)
            if m.ns != 'tracked_objects':
                continue

            key = (m.ns, m.id)
            x = m.pose.position.x
            y = m.pose.position.y
            dist = math.hypot(x, y)

            # track closest object for emergency stop
            if dist > 0.1 and dist < min_dist:
                min_dist = dist

            # skip objects outside detection range
            if dist < self.min_detection_dist or dist > self.max_detection_dist:
                continue

            # compute velocity with ego-motion compensation
            if key in self.last_positions:
                prev = self.last_positions[key]
                dt = now - prev['t']

                if 0.03 < dt < 1.0:  # valid time delta
                    # raw measured velocity (apparent motion in lidar_link frame)
                    vx_meas = (x - prev['x']) / dt
                    vy_meas = (y - prev['y']) / dt

                    # ego-motion compensation:
                    # when robot moves forward, static objects appear to move backward
                    # when robot rotates, static objects appear to move tangentially
                    # compensated velocity = measured + robot_motion_effect
                    vx_comp = vx_meas + self.robot_vx - self.robot_wz * y
                    vy_comp = vy_meas + self.robot_wz * x

                    # smooth the compensated velocity
                    vx_smooth = (self.velocity_alpha * vx_comp +
                                (1 - self.velocity_alpha) * prev.get('vx_smooth', 0.0))
                    vy_smooth = (self.velocity_alpha * vy_comp +
                                (1 - self.velocity_alpha) * prev.get('vy_smooth', 0.0))

                    # compute speed and approach speed
                    speed = math.hypot(vx_smooth, vy_smooth)

                    # sanity check - ignore unrealistic speeds
                    if speed > self.max_reasonable_speed:
                        self.last_positions[key] = {
                            'x': x, 'y': y, 't': now,
                            'vx_smooth': 0.0, 'vy_smooth': 0.0
                        }
                        continue

                    # approach speed (positive = moving toward robot)
                    if dist > 1e-3:
                        approach_speed = -(x * vx_smooth + y * vy_smooth) / dist
                    else:
                        approach_speed = 0.0

                    # is this object approaching us fast enough to be a threat?
                    if approach_speed > self.approach_speed_threshold:
                        ttc = dist / max(approach_speed, 1e-3)
                        if candidate_ttc is None or ttc < candidate_ttc:
                            candidate_ttc = ttc
                            candidate_dist = dist
                            threat_found = True

                    self.last_positions[key] = {
                        'x': x, 'y': y, 't': now,
                        'vx_smooth': vx_smooth, 'vy_smooth': vy_smooth
                    }
                else:
                    # time delta out of range, reset velocity
                    self.last_positions[key] = {
                        'x': x, 'y': y, 't': now,
                        'vx_smooth': 0.0, 'vy_smooth': 0.0
                    }
            else:
                # new track
                self.last_positions[key] = {
                    'x': x, 'y': y, 't': now,
                    'vx_smooth': 0.0, 'vy_smooth': 0.0
                }

        # update closest object distance
        self.closest_object_dist = min_dist

        # update threat state (require consecutive detections)
        if threat_found:
            self.consecutive_threats += 1
            self.last_threat_time = now
            if self.consecutive_threats >= self.min_consecutive:
                self.current_ttc = candidate_ttc
                self.current_dist = candidate_dist
                self.get_logger().info(
                    f"Moving object: dist={candidate_dist:.2f}m, ttc={candidate_ttc:.2f}s"
                )
        else:
            self.consecutive_threats = 0
            if now - self.last_threat_time > self.threat_timeout:
                self.current_ttc = None
                self.current_dist = None

    def cmd_raw_callback(self, cmd: Twist):
        # store raw command for timer-based processing
        self.last_raw_cmd = cmd
        self.last_raw_cmd_time = time.time()

    def safety_timer_callback(self):
        # continuous safety check at 20Hz
        # this allows stopping even if no new commands arrive
        now = time.time()

        # if no recent command, gradually stop
        if now - self.last_raw_cmd_time > self.cmd_timeout:
            safe_cmd = Twist()
            self.cmd_safe_pub.publish(safe_cmd)
            return

        # start with last raw command
        safe_cmd = Twist()
        safe_cmd.linear.x = self.last_raw_cmd.linear.x
        safe_cmd.linear.y = self.last_raw_cmd.linear.y
        safe_cmd.linear.z = self.last_raw_cmd.linear.z
        safe_cmd.angular.x = self.last_raw_cmd.angular.x
        safe_cmd.angular.y = self.last_raw_cmd.angular.y
        safe_cmd.angular.z = self.last_raw_cmd.angular.z

        # EMERGENCY STOP: if any object is too close
        if self.closest_object_dist < self.emergency_stop_dist:
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.angular.z = 0.0
            self.get_logger().warn(
                f"EMERGENCY STOP: object at {self.closest_object_dist:.2f}m"
            )
            self.cmd_safe_pub.publish(safe_cmd)
            return

        # TTC-based safety for moving objects
        if self.current_ttc is not None:
            ttc = self.current_ttc
            dist = self.current_dist or 999

            # hard stop if collision imminent
            if ttc < self.ttc_stop_threshold:
                safe_cmd.linear.x = 0.0
                safe_cmd.linear.y = 0.0
                safe_cmd.angular.z *= 0.3  # allow some rotation
                self.get_logger().warn(
                    f"STOP: ttc={ttc:.2f}s, dist={dist:.2f}m"
                )
                self.cmd_safe_pub.publish(safe_cmd)
                return

            # slow down if TTC is low
            if ttc < self.ttc_slow_threshold:
                factor = (ttc - self.ttc_stop_threshold) / (
                    self.ttc_slow_threshold - self.ttc_stop_threshold
                )
                factor = max(0.2, min(1.0, factor))
                safe_cmd.linear.x *= factor
                self.get_logger().info(
                    f"Slowing: ttc={ttc:.2f}s, factor={factor:.2f}"
                )

        self.cmd_safe_pub.publish(safe_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
