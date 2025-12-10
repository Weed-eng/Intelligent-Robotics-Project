# adaptive safety v8.0 - distance-first safety
#
# KEY PRINCIPLE: DISTANCE IS KING
# No matter what trajectory analysis says, if ped is too close, react!
#
# v7.0 BUG: in_path_threshold (0.30m) was LESS than collision_radius (0.37m)
# Robot would STOP when "not in path" even if ped was 0.29m away = COLLISION!
# Also: t_closest > 0 requirement caused STOP when ped passed closest point
#
# v8.0 FIX: Priority-based decision with distance override
#
# PRIORITY 1: EMERGENCY (dist < 0.45m)
#    -> ALWAYS react regardless of trajectory - collision imminent
#    -> Backup if ped ahead, forward if ped behind/side
#
# PRIORITY 2: CRITICAL (dist < 0.60m)
#    -> Active evasion if in_path (NO t_closest requirement!)
#    -> Slow significantly if not in path
#
# PRIORITY 3: CAUTION (dist < 1.2m, in_path, t_closest > 0)
#    -> Standard trajectory-based evasion
#
# PRIORITY 4: SAFE
#    -> STOP/SLOW if close, pass through if far

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

        self.tracks_sub = self.create_subscription(
            MarkerArray, '/tracked_objects', self.tracks_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # robot state
        self.robot_vx = 0.0
        self.robot_wz = 0.0
        self.last_cmd = Twist()
        self.last_cmd_time = 0.0

        # safety timer at 20Hz
        self.timer = self.create_timer(0.05, self.safety_callback)

        # tracked pedestrians
        self.pedestrians = []

        # robot parameters
        self.robot_radius = 0.12
        self.ped_radius = 0.25
        self.collision_radius = self.robot_radius + self.ped_radius  # 0.37m

        # thresholds - CORRECTED for robot(0.12) + ped(0.25) radii
        self.in_path_threshold = 0.40    # robot in path (accounts for both radii)
        self.safe_threshold = 0.55       # clearly out of path (hysteresis)
        self.watch_dist = 1.2            # monitor distance
        self.backup_dist = 0.45          # backup if closer than this
        self.emergency_dist = 0.45       # MUST react - collision imminent
        self.critical_dist = 0.60        # active evasion required

        # velocity options
        self.forward_vel = 0.22
        self.backup_vel = -0.12

        # evasion state (hysteresis)
        self.evading = False
        self.evade_start_time = 0
        self.evade_direction = 0  # angular velocity direction

        # commitment
        self.commit_start = 0
        self.commit_duration = 0.6  # longer commitment

        # velocity smoothing (anti-jitter)
        self.smooth_vx = 0.0
        self.smooth_wz = 0.0
        self.smooth_alpha = 0.4  # balance responsiveness vs smoothness

        self.get_logger().info('Adaptive safety v8.0: distance-first safety')

    def odom_callback(self, msg):
        self.robot_vx = msg.twist.twist.linear.x
        self.robot_wz = msg.twist.twist.angular.z

    def tracks_callback(self, msg):
        self.pedestrians = []
        positions = {}
        velocities = {}

        for m in msg.markers:
            if m.ns == 'tracked_objects':
                positions[m.id] = (m.pose.position.x, m.pose.position.y)
            elif m.ns == 'velocity' and len(m.points) >= 2:
                ped_id = m.id - 10000
                vx = m.points[1].x - m.points[0].x
                vy = m.points[1].y - m.points[0].y
                velocities[ped_id] = (vx, vy)

        for ped_id, (px, py) in positions.items():
            vx, vy = velocities.get(ped_id, (0.0, 0.0))
            self.pedestrians.append({'x': px, 'y': py, 'vx': vx, 'vy': vy})

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def perp_dist_to_trajectory(self, px, py, vx, vy):
        """Perpendicular distance from robot to pedestrian's trajectory line."""
        speed = math.hypot(vx, vy)
        if speed < 0.05:
            return math.hypot(px, py)
        return abs(px * vy - py * vx) / speed

    def time_to_closest_point(self, px, py, vx, vy):
        """Time until pedestrian reaches closest point to robot."""
        speed_sq = vx * vx + vy * vy
        if speed_sq < 0.001:
            return float('inf')
        return -(px * vx + py * vy) / speed_sq

    def compute_escape_steer(self, px, py, vx, vy):
        """
        Compute which direction to steer to escape pedestrian's path.
        Returns angular velocity: positive = turn left, negative = turn right.
        Goal: move perpendicular AWAY from pedestrian's trajectory.
        """
        speed = math.hypot(vx, vy)
        if speed < 0.05:
            # Pedestrian stationary - steer away from their position
            return -0.7 if py > 0 else 0.7

        # Cross product determines which side of trajectory we're on
        # cross > 0: robot is LEFT of pedestrian's trajectory
        # cross < 0: robot is RIGHT of pedestrian's trajectory
        cross = px * vy - py * vx

        # Steer to increase distance from trajectory
        # If we're left of trajectory, go more left (positive angular)
        # If we're right of trajectory, go more right (negative angular)
        return 0.7 if cross > 0 else -0.7

    def safety_callback(self):
        now = time.time()

        # No nav2 command - stop
        if now - self.last_cmd_time > 0.5:
            self.cmd_pub.publish(Twist())
            self.evading = False
            return

        # Start with nav2 command
        cmd = Twist()
        cmd.linear.x = self.last_cmd.linear.x
        cmd.angular.z = self.last_cmd.angular.z

        # Find the most dangerous pedestrian
        threat = None
        min_dist = float('inf')

        for p in self.pedestrians:
            px, py = p['x'], p['y']
            vx, vy = p['vx'], p['vy']
            dist = math.hypot(px, py)

            # Skip if inside robot or too far
            if dist < self.robot_radius or dist > self.watch_dist:
                continue

            # Pedestrian angle (0° = ahead, 90° = side, 180° = behind)
            angle = abs(math.degrees(math.atan2(py, px)))

            # Time to closest approach
            t_closest = self.time_to_closest_point(px, py, vx, vy)

            # Skip departing pedestrians (already passed) unless very close
            if t_closest < 0 and dist > self.backup_dist:
                continue

            # Skip pedestrians behind that are moving away
            if angle > 120:
                # Check if approaching
                rel_vx = vx - self.robot_vx
                approaching = (px * rel_vx + py * vy) < -0.01
                if not approaching:
                    continue

            # Calculate perpendicular distance to trajectory
            perp_dist = self.perp_dist_to_trajectory(px, py, vx, vy)

            # Select closest threat
            if dist < min_dist:
                min_dist = dist
                threat = {
                    'px': px, 'py': py, 'vx': vx, 'vy': vy,
                    'dist': dist, 'angle': angle, 'perp_dist': perp_dist,
                    't_closest': t_closest
                }

        # No threat - pass through nav2 command
        if threat is None:
            # Clear evading state if pedestrian is gone
            if self.evading and (now - self.evade_start_time) > 1.0:
                self.evading = False
            self.cmd_pub.publish(cmd)
            return

        # Extract threat info
        px, py = threat['px'], threat['py']
        vx, vy = threat['vx'], threat['vy']
        dist = threat['dist']
        angle = threat['angle']
        perp_dist = threat['perp_dist']
        t_closest = threat['t_closest']

        # Determine if robot is in pedestrian's path
        # Use hysteresis: enter evade at 0.30m, exit at 0.50m
        if self.evading:
            robot_in_path = perp_dist < self.safe_threshold
        else:
            robot_in_path = perp_dist < self.in_path_threshold

        # ============================================================
        # PRIORITY 1: EMERGENCY - collision imminent (dist < 0.45m)
        # Distance overrides ALL trajectory analysis!
        # ============================================================
        if dist < self.emergency_dist:
            if not self.evading:
                self.evading = True
                self.evade_start_time = now
                self.evade_direction = self.compute_escape_steer(px, py, vx, vy)

            if angle < 60:
                # ped ahead - backup immediately
                cmd.linear.x = self.backup_vel
                cmd.angular.z = -0.5 if py > 0 else 0.5
                self.get_logger().error(
                    f'EMERGENCY-BACK: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                )
            else:
                # ped to side/behind - go forward fast
                cmd.linear.x = self.forward_vel
                cmd.angular.z = self.evade_direction * 0.6
                self.get_logger().error(
                    f'EMERGENCY-FWD: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                )

        # ============================================================
        # PRIORITY 2: CRITICAL - very close (dist < 0.60m)
        # Don't require t_closest > 0 - react to proximity!
        # ============================================================
        elif dist < self.critical_dist:
            if robot_in_path:
                # in path at close range - evade regardless of t_closest
                if not self.evading:
                    self.evading = True
                    self.evade_start_time = now
                    self.evade_direction = self.compute_escape_steer(px, py, vx, vy)

                if angle > 90:
                    cmd.linear.x = self.forward_vel
                    cmd.angular.z = self.evade_direction * 0.5
                    self.get_logger().warn(
                        f'CRITICAL-FWD(behind): d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                    )
                elif angle > 40:
                    cmd.linear.x = self.forward_vel
                    cmd.angular.z = self.evade_direction * 0.7
                    self.get_logger().warn(
                        f'CRITICAL-FWD(side): d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                    )
                else:
                    cmd.linear.x = self.backup_vel
                    cmd.angular.z = -0.5 if py > 0 else 0.5
                    self.get_logger().warn(
                        f'CRITICAL-BACK: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                    )
            else:
                # not in path but still close - slow significantly, don't stop
                self.evading = False
                cmd.linear.x = self.last_cmd.linear.x * 0.25
                cmd.angular.z = self.last_cmd.angular.z * 0.3
                self.get_logger().info(
                    f'CRITICAL-SLOW: d={dist:.2f} perp={perp_dist:.2f}'
                )

        # ============================================================
        # PRIORITY 3: CAUTION - trajectory analysis OK at this range
        # Robot IS in pedestrian's path -> MUST MOVE
        # ============================================================
        elif robot_in_path and t_closest > 0:
            if not self.evading:
                self.evading = True
                self.evade_start_time = now
                self.evade_direction = self.compute_escape_steer(px, py, vx, vy)
                self.commit_start = now

            if angle > 90:
                cmd.linear.x = self.forward_vel
                cmd.angular.z = self.evade_direction * 0.5
                self.get_logger().info(
                    f'EVADE-FWD(behind): d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                )
            elif angle > 40:
                cmd.linear.x = self.forward_vel
                cmd.angular.z = self.evade_direction * 0.8
                self.get_logger().info(
                    f'EVADE-FWD(side): d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                )
            else:
                if dist < self.backup_dist or t_closest < 0.8:
                    cmd.linear.x = self.backup_vel
                    steer = -0.5 if py > 0 else 0.5
                    cmd.angular.z = steer
                    self.get_logger().warn(
                        f'EVADE-BACK: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                    )
                else:
                    cmd.linear.x = self.forward_vel
                    cmd.angular.z = self.evade_direction * 0.8
                    self.get_logger().info(
                        f'EVADE-FWD(ahead): d={dist:.2f} perp={perp_dist:.2f} t={t_closest:.1f}s'
                    )

        # ============================================================
        # PRIORITY 4: NOT in path or ped departing -> STOP/SLOW
        # ============================================================
        else:
            self.evading = False

            if dist < 0.70:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f'STOP: d={dist:.2f} perp={perp_dist:.2f} - letting ped pass'
                )
            elif dist < self.watch_dist:
                factor = max(0.2, (dist - 0.70) / 0.50)
                cmd.linear.x = self.last_cmd.linear.x * min(factor, 0.4)
                self.get_logger().info(
                    f'SLOW: d={dist:.2f} perp={perp_dist:.2f}'
                )

        # Clamp velocities
        cmd.linear.x = max(-0.15, min(0.22, cmd.linear.x))
        cmd.angular.z = max(-1.2, min(1.2, cmd.angular.z))

        # Apply smoothing to prevent jitter
        self.smooth_vx = self.smooth_alpha * cmd.linear.x + (1 - self.smooth_alpha) * self.smooth_vx
        self.smooth_wz = self.smooth_alpha * cmd.angular.z + (1 - self.smooth_alpha) * self.smooth_wz
        cmd.linear.x = self.smooth_vx
        cmd.angular.z = self.smooth_wz

        self.cmd_pub.publish(cmd)


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
