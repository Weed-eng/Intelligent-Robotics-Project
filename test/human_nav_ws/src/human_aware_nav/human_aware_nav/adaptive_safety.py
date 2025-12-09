# adaptive safety - trajectory-based pedestrian avoidance
# uses Closest Point of Approach (CPA) to predict actual collisions
# only intervenes when paths will intersect, not just when pedestrians are nearby

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
        self.last_track_time = time.time()

        # robot parameters (turtlebot3 burger)
        self.robot_radius = 0.12
        self.human_radius = 0.15
        self.safety_margin = 0.08
        self.collision_radius = self.robot_radius + self.human_radius + self.safety_margin

        self.robot_max_speed = 0.22
        self.robot_max_turn = 1.5

        # CPA-based thresholds
        self.cpa_danger = 0.35      # if CPA < this, paths will collide
        self.cpa_caution = 0.50     # if CPA < this, slow down
        self.max_reaction_time = 3.0  # only react to collisions within 3s
        self.emergency_dist = 0.25  # immediate danger zone

        self.get_logger().info(
            f'Adaptive safety (CPA-based): collision_radius={self.collision_radius:.2f}m, '
            f'cpa_danger={self.cpa_danger:.2f}m'
        )

    def odom_callback(self, msg):
        self.robot_vx = msg.twist.twist.linear.x
        self.robot_wz = msg.twist.twist.angular.z

    def tracks_callback(self, msg):
        """Extract pedestrian positions and velocities."""
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
            self.pedestrians.append({
                'x': px, 'y': py, 'vx': vx, 'vy': vy, 'id': ped_id
            })

        self.last_track_time = time.time()

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def compute_cpa(self, px, py, vx, vy):
        """
        Compute Closest Point of Approach (CPA) between robot and pedestrian.

        In lidar frame:
        - Robot is at origin (0, 0) and effectively stationary
        - Pedestrian velocity is already relative to robot (lidar moves with robot)
        - Pedestrian at (px, py) moving at (vx, vy)
        - Future position: (px + vx*t, py + vy*t)

        Returns: (t_cpa, cpa_dist)
        - t_cpa: time until closest approach (negative = already passed)
        - cpa_dist: minimum distance at closest approach
        """
        # relative velocity magnitude squared
        v_sq = vx * vx + vy * vy

        if v_sq < 0.001:  # pedestrian nearly stationary relative to robot
            # CPA is current position
            return 0.0, math.hypot(px, py)

        # time to closest approach: t = -dot(pos, vel) / |vel|^2
        t_cpa = -(px * vx + py * vy) / v_sq

        if t_cpa < 0:
            # closest approach was in the past, return current distance
            return t_cpa, math.hypot(px, py)

        # position at time of closest approach
        cpa_x = px + vx * t_cpa
        cpa_y = py + vy * t_cpa
        cpa_dist = math.hypot(cpa_x, cpa_y)

        return t_cpa, cpa_dist

    def safety_callback(self):
        now = time.time()

        # no command from Nav2 - stop
        if now - self.last_cmd_time > 0.5:
            self.cmd_pub.publish(Twist())
            return

        # start with Nav2's command
        cmd = Twist()
        cmd.linear.x = self.last_cmd.linear.x
        cmd.angular.z = self.last_cmd.angular.z

        # find the most dangerous pedestrian based on CPA
        most_urgent_tcpa = float('inf')
        threat_ped = None
        threat_level = 'NONE'

        for p in self.pedestrians:
            px, py = p['x'], p['y']
            vx, vy = p['vx'], p['vy']
            dist = math.hypot(px, py)

            # skip pedestrians behind us
            if px < -0.15:
                continue

            # EMERGENCY: very close regardless of trajectory
            if dist < self.emergency_dist:
                threat_level = 'EMERGENCY'
                threat_ped = p
                threat_ped['dist'] = dist
                break

            # compute closest point of approach
            t_cpa, cpa_dist = self.compute_cpa(px, py, vx, vy)

            # skip if CPA is in the past (pedestrian already passed)
            if t_cpa < 0:
                continue

            # skip if CPA is too far in the future
            if t_cpa > self.max_reaction_time:
                continue

            # skip if paths won't come close (no collision risk)
            if cpa_dist > self.cpa_caution:
                continue

            # this pedestrian is a potential threat - check if most urgent
            # urgency = how soon AND how close
            urgency = t_cpa + cpa_dist  # lower = more urgent

            if urgency < most_urgent_tcpa + (threat_ped['cpa'] if threat_ped else float('inf')):
                most_urgent_tcpa = t_cpa
                threat_ped = p
                threat_ped['t_cpa'] = t_cpa
                threat_ped['cpa'] = cpa_dist
                threat_ped['dist'] = dist

                # determine threat level based on CPA and time
                if cpa_dist < self.cpa_danger:
                    if t_cpa < 1.0:
                        threat_level = 'STOP'
                    elif t_cpa < 2.0:
                        threat_level = 'SLOW'
                    else:
                        threat_level = 'CAUTION'
                elif cpa_dist < self.cpa_caution:
                    if t_cpa < 1.5:
                        threat_level = 'SLOW'
                    else:
                        threat_level = 'CAUTION'

        # handle threat
        if threat_level == 'EMERGENCY' and threat_ped:
            px, py = threat_ped['x'], threat_ped['y']
            dist = threat_ped['dist']

            if px > 0:
                # pedestrian in front - backup and turn away
                cmd.linear.x = -0.10
                cmd.angular.z = 1.2 if py > 0 else -1.2
                self.get_logger().warn(f'EMERGENCY: ped at {dist:.2f}m - backing up')
            else:
                # pedestrian behind - accelerate forward
                cmd.linear.x = self.robot_max_speed
                cmd.angular.z = 0.5 if py > 0 else -0.5
                self.get_logger().warn(f'EMERGENCY ESCAPE: ped behind at {dist:.2f}m')

        elif threat_level == 'STOP' and threat_ped:
            # collision imminent - full stop
            cmd.linear.x = 0.0
            t_cpa = threat_ped['t_cpa']
            cpa = threat_ped['cpa']
            dist = threat_ped['dist']
            self.get_logger().info(
                f'STOP: collision in {t_cpa:.1f}s, CPA={cpa:.2f}m, dist={dist:.2f}m'
            )

        elif threat_level == 'SLOW' and threat_ped:
            # collision likely - slow down proportionally
            t_cpa = threat_ped['t_cpa']
            cpa = threat_ped['cpa']
            dist = threat_ped['dist']

            # slow factor based on time to collision
            slow_factor = min(1.0, t_cpa / 2.0)  # full speed at 2s, zero at 0s
            slow_factor = max(0.2, slow_factor)  # minimum 20% speed
            cmd.linear.x = cmd.linear.x * slow_factor

            self.get_logger().info(
                f'SLOW: collision in {t_cpa:.1f}s, CPA={cpa:.2f}m, dist={dist:.2f}m, factor={slow_factor:.1f}'
            )

        elif threat_level == 'CAUTION' and threat_ped:
            # potential collision - minor slowdown
            t_cpa = threat_ped['t_cpa']
            cpa = threat_ped['cpa']
            slow_factor = 0.7  # 70% speed
            cmd.linear.x = cmd.linear.x * slow_factor

        # clamp velocities
        cmd.linear.x = max(-0.15, min(self.robot_max_speed, cmd.linear.x))
        cmd.angular.z = max(-self.robot_max_turn, min(self.robot_max_turn, cmd.angular.z))

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
