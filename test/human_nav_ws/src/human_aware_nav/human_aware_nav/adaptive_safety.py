# adaptive safety v7.0 - proper evasion logic
#
# KEY PRINCIPLE: When robot IS in pedestrian's path, it MUST MOVE.
# Stopping when you're in someone's path doesn't help - you'll still be hit!
#
# LOGIC:
# 1) Scan for approaching pedestrians
# 2) IF robot IN pedestrian's path (perp_dist < 0.30m):
#    -> MUST EVADE (physically move out of the way)
#    -> Choose based on pedestrian angle:
#       - Behind (>90°): go FORWARD
#       - Side (45-90°): go FORWARD + strong steer away
#       - Ahead (<45°): BACKUP if close, else FORWARD + steer
#    -> NO STOPPING when in path!
#
# 3) IF robot NOT in path (perp_dist >= 0.30m):
#    -> STOP/SLOW and let pedestrian pass
#
# 4) Hysteresis: once evading, stay evading until clearly safe

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

        # thresholds
        self.in_path_threshold = 0.30    # robot is IN pedestrian's path
        self.safe_threshold = 0.50       # clearly out of path (hysteresis)
        self.watch_dist = 1.2            # monitor distance
        self.backup_dist = 0.45          # backup if closer than this

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

        self.get_logger().info('Adaptive safety v7.0: proper evasion (no stopping in path)')

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
        # CASE 1: Robot IS in pedestrian's path -> MUST MOVE
        # No stopping allowed! Robot must physically get out of the way.
        # ============================================================
        if robot_in_path and t_closest > 0:
            if not self.evading:
                self.evading = True
                self.evade_start_time = now
                self.evade_direction = self.compute_escape_steer(px, py, vx, vy)
                self.commit_start = now

            # Decide evasion action based on pedestrian position
            if angle > 90:
                # Pedestrian is BEHIND - go FORWARD (away from them)
                cmd.linear.x = self.forward_vel
                cmd.angular.z = self.evade_direction * 0.5
                self.get_logger().info(
                    f'EVADE-FWD(behind): d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                )

            elif angle > 40:
                # Pedestrian is to the SIDE (40-90°) - FORWARD + strong steer
                cmd.linear.x = self.forward_vel
                cmd.angular.z = self.evade_direction * 0.8  # strong steer
                self.get_logger().info(
                    f'EVADE-FWD(side): d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                )

            else:
                # Pedestrian is AHEAD (<40°)
                if dist < self.backup_dist or t_closest < 0.8:
                    # Too close or not enough time - BACKUP
                    cmd.linear.x = self.backup_vel
                    # Steer away while backing up (reverse direction)
                    steer = -0.5 if py > 0 else 0.5
                    cmd.angular.z = steer
                    self.get_logger().warn(
                        f'EVADE-BACK: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                    )
                else:
                    # Some distance - try FORWARD + aggressive steer
                    cmd.linear.x = self.forward_vel
                    cmd.angular.z = self.evade_direction * 0.8
                    self.get_logger().info(
                        f'EVADE-FWD(ahead): d={dist:.2f} perp={perp_dist:.2f} t={t_closest:.1f}s'
                    )

        # ============================================================
        # CASE 2: Robot NOT in pedestrian's path -> STOP/SLOW
        # Let the pedestrian pass safely.
        # ============================================================
        else:
            self.evading = False

            if dist < 0.70:
                # Close - stop completely
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f'STOP: d={dist:.2f} perp={perp_dist:.2f} - letting ped pass'
                )
            elif dist < self.watch_dist:
                # Slow down
                factor = max(0.2, (dist - 0.70) / 0.50)
                cmd.linear.x = self.last_cmd.linear.x * min(factor, 0.4)
                self.get_logger().info(
                    f'SLOW: d={dist:.2f} perp={perp_dist:.2f}'
                )

        # Clamp velocities
        cmd.linear.x = max(-0.15, min(0.22, cmd.linear.x))
        cmd.angular.z = max(-1.2, min(1.2, cmd.angular.z))
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
