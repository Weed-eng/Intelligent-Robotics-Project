import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Time as RosTime

import math
import time


class AdaptiveSafety(Node):
    """
    Adaptive safety module:
    - Subscribes to /tracked_objects (MarkerArray from Kalman tracker)
    - Subscribes to /cmd_vel_raw (Twist from DWA/teleop)
    - Publishes safe /cmd_vel (Twist) to be sent to the robot
    """

    def __init__(self):
        super().__init__('adaptive_safety')

        # Subscribe to tracked objects
        self.tracks_sub = self.create_subscription(
            MarkerArray,
            '/tracked_objects',
            self.tracked_objects_callback,
            10
        )

        # Subscribe to raw velocity commands
        self.cmd_raw_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_raw_callback,
            10
        )

        # Publish filtered/safe velocity commands
        self.cmd_safe_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Store last positions of tracked objects to estimate velocity
        # key: (ns, id) -> dict with x, y, t
        self.last_positions = {}

        # Store current "threat level"
        self.current_min_ttc = None
        self.current_min_dist = None
        self.last_threat_update_time = time.time()

        # Safety parameters (tune later)
        self.slow_down_radius = 1.5      # [m] start slowing down
        self.hard_stop_radius = 0.6      # [m] immediate stop zone
        self.approach_speed_threshold = 0.1  # [m/s] must be moving toward robot faster than this
        self.ttc_stop_threshold = 1.0    # [s] stop if time-to-collision is below this
        self.ttc_slow_threshold = 2.5    # [s] start slowing if TTC is below this

        # If no threats for some time, reset threat state
        self.threat_timeout = 1.0  # [s]

        self.get_logger().info("Adaptive safety module started.")

    # -------------------------------
    #   Tracking / threat evaluation
    # -------------------------------
    def tracked_objects_callback(self, msg: MarkerArray):
        """
        Called whenever Kalman tracker publishes /tracked_objects.
        We estimate velocity per object using position differences,
        then compute distance, approach speed, and TTC.
        """
        now = time.time()
        self.current_min_ttc = None
        self.current_min_dist = None
        threat_found = False

        for m in msg.markers:
            # We only care about the position markers, not velocity arrows.
            if m.ns != 'tracked_objects':
                continue

            key = (m.ns, m.id)
            x = m.pose.position.x
            y = m.pose.position.y
            dist = math.hypot(x, y)

            # Retrieve previous position for this track
            if key in self.last_positions:
                prev = self.last_positions[key]
                dt = now - prev['t']
                if dt > 0.05:  # avoid division by very small numbers
                    vx = (x - prev['x']) / dt
                    vy = (y - prev['y']) / dt

                    # Compute radial approach speed (projected onto line from robot->object)
                    if dist > 1e-3:
                        approach_speed = -(x * vx + y * vy) / dist
                    else:
                        approach_speed = 0.0

                    # Only consider threats moving toward robot
                    if approach_speed > self.approach_speed_threshold:
                        ttc = dist / max(approach_speed, 1e-3)

                        if (self.current_min_ttc is None) or (ttc < self.current_min_ttc):
                            self.current_min_ttc = ttc
                            self.current_min_dist = dist
                            threat_found = True

            # Update stored position
            self.last_positions[key] = {'x': x, 'y': y, 't': now}

        if threat_found:
            self.last_threat_update_time = now
            self.get_logger().debug(
                f"Threat detected: min_dist={self.current_min_dist:.2f} m, "
                f"min_ttc={self.current_min_ttc:.2f} s"
            )
        else:
            # If no threat in this message, we may clear after timeout
            if now - self.last_threat_update_time > self.threat_timeout:
                self.current_min_ttc = None
                self.current_min_dist = None

    # -------------------------------
    #   Command filtering
    # -------------------------------
    def cmd_raw_callback(self, cmd: Twist):
        """
        Called when DWA/teleop publishes /cmd_vel_raw.
        We apply safety logic and then publish to /cmd_vel.
        """
        safe_cmd = Twist()
        safe_cmd.linear.x = cmd.linear.x
        safe_cmd.linear.y = cmd.linear.y
        safe_cmd.linear.z = cmd.linear.z
        safe_cmd.angular.x = cmd.angular.x
        safe_cmd.angular.y = cmd.angular.y
        safe_cmd.angular.z = cmd.angular.z

        # If no current threat, pass through
        if self.current_min_dist is None or self.current_min_ttc is None:
            self.cmd_safe_pub.publish(safe_cmd)
            return

        d = self.current_min_dist
        ttc = self.current_min_ttc

        # 1) Hard stop region
        if d < self.hard_stop_radius or ttc < self.ttc_stop_threshold:
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            # Allow turning slowly in place, but you can also zero angular.z if you want a full freeze
            safe_cmd.angular.z *= 0.5

            self.get_logger().warn(
                f"EMERGENCY BRAKE: dist={d:.2f} m, ttc={ttc:.2f} s → stopping."
            )
            self.cmd_safe_pub.publish(safe_cmd)
            return

        # 2) Slow down region
        if d < self.slow_down_radius or ttc < self.ttc_slow_threshold:
            # Compute a scaling factor based on how "dangerous" it is
            # Closer / smaller TTC → stronger slowdown
            dist_factor = max(0.0, min(1.0, (d - self.hard_stop_radius) /
                                       max(self.slow_down_radius - self.hard_stop_radius, 1e-3)))
            ttc_factor = max(0.0, min(1.0, (ttc - self.ttc_stop_threshold) /
                                      max(self.ttc_slow_threshold - self.ttc_stop_threshold, 1e-3)))

            slowdown_factor = min(dist_factor, ttc_factor)  # be conservative
            slowdown_factor = 0.2 + 0.8 * slowdown_factor   # never go completely to zero here

            safe_cmd.linear.x *= slowdown_factor

            self.get_logger().info(
                f"Slowing down: dist={d:.2f} m, ttc={ttc:.2f} s, "
                f"factor={slowdown_factor:.2f}"
            )

        # 3) Publish safe command
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
