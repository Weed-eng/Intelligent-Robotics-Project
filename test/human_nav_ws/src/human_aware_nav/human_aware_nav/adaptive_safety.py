# adaptive safety - predictive pedestrian avoidance layer
#
# distance-first priority with aimed check
# only backup if ped is aimed at robot, if crossing perpendicular just yield
#
# priority 1 - emergency (dist < 0.45m)
#    if aimed and angle < 60: backup (with timeout)
#    else: yield (creep 0.03 m/s)
#
# priority 2 - critical (dist < 0.60m)
#    if in_path and aimed and angle < 40: backup
#    else if in_path: fwd + steer
#    else: slow
#
# priority 3 - caution (dist < 1.2m, in_path, t_closest > 0)
#    if aimed: backup or fwd based on distance/time
#    else: fwd + steer
#
# priority 4 - safe
#    creep if close, slow if far

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
        self.in_path_threshold = 0.35 
        self.safe_threshold = 0.55       # clearly out of path (hysteresis)
        self.watch_dist = 1.2            # monitor distance
        self.backup_dist = 0.45          # backup if closer than this
        self.emergency_dist = 0.45       # must react - collision imminent
        self.critical_dist = 0.60        # active evasion required

        # velocity options
        self.forward_vel = 0.18 
        self.backup_vel = -0.15
        self.creep_vel = 0.03

        # evasion state (hysteresis)
        self.evading = False
        self.evade_start_time = 0
        self.evade_direction = 0  # angular velocity direction
        self.backup_start_time = 0.0
        self.backup_timeout = 3.5  # seconds
        self.dodge_direction = 0.0
        self.dodge_commit_time = 0.0
        self.dodge_timeout = 3.0  # stay committed to an actionfor 3 seconds
        self.current_action = None  # 'BACKUP', 'YIELD', 'EVADE', 'SLOW', 'CREEP'
        self.action_start_time = 0.0
        self.action_min_duration = 0.6
        self.last_threat_dist = None
        self.last_threat_time = 0.0
        self.was_aimed = False

        # velocity smoothing (anti-jitter)
        self.smooth_vx = 0.0
        self.smooth_wz = 0.0
        self.smooth_alpha = 0.55
        self.last_safe_vx = 0.0
        self.last_safe_wz = 0.0

        self.get_logger().info('Adaptive safety v8.5: continuous jitter fixes')

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

    # perpendicular distance from robot to pedestrian's trajectory line
    def perp_dist_to_trajectory(self, px, py, vx, vy):
        speed = math.hypot(vx, vy)
        if speed < 0.05:
            return math.hypot(px, py)
        return abs(px * vy - py * vx) / speed

    # time until pedestrian reaches closest point to robot
    def time_to_closest_point(self, px, py, vx, vy):
        speed_sq = vx * vx + vy * vy
        if speed_sq < 0.001:
            return float('inf')
        return -(px * vx + py * vy) / speed_sq

    # compute which direction to steer to escape pedestrian's path
    # returns angular velocity: positive = turn left, negative = turn right
    # goal: move perpendicular away from pedestrian's trajectory
    def compute_escape_steer(self, px, py, vx, vy):
        speed = math.hypot(vx, vy)
        if speed < 0.05:
            # pedestrian stationary - steer away from their position
            return -0.7 if py > 0 else 0.7

        # cross product determines which side of trajectory we're on
        # cross > 0: robot is left of pedestrian's trajectory
        # cross < 0: robot is right of pedestrian's trajectory
        cross = px * vy - py * vx

        # steer to increase distance from trajectory
        # if we're left of trajectory, go more left (positive angular)
        # if we're right of trajectory, go more right (negative angular)
        return 0.7 if cross > 0 else -0.7

    # check if pedestrian is walking toward robot position
    # uses hysteresis to prevent rapid aimed/not-aimed oscillation
    def is_aimed_at_robot(self, px, py, vx, vy):
        ped_speed = math.hypot(vx, vy)
        if ped_speed < 0.05:
            self.was_aimed = False
            return False  # stationary ped - not aimed

        dist = math.hypot(px, py)
        if dist < 0.01:
            self.was_aimed = True
            return True  # at collision

        # dot(velocity, -position) > 0 means walking toward robot
        toward_robot = vx * (-px) + vy * (-py)
        aimed_score = toward_robot / (dist * ped_speed)

        # рysteresis to prevent oscillation
        if self.was_aimed:
            self.was_aimed = aimed_score > 0.15
        else:
            self.was_aimed = aimed_score > 0.45

        return self.was_aimed

    def get_dodge_steer(self, py, now):
        if self.dodge_direction == 0 or (now - self.dodge_commit_time) > self.dodge_timeout:
            self.dodge_direction = -1.0 if py > 0 else 1.0
            self.dodge_commit_time = now
        return self.dodge_direction

    def set_action(self, new_action, now):
        if self.current_action == new_action:
            return True 

        if self.current_action is not None:
            if (now - self.action_start_time) < self.action_min_duration:
                return False

        self.current_action = new_action
        self.action_start_time = now
        return True

    def safety_callback(self):
        now = time.time()

        if now - self.last_cmd_time > 0.5:
            self.cmd_pub.publish(Twist())
            self.evading = False
            return

        cmd = Twist()
        cmd.linear.x = self.last_cmd.linear.x
        cmd.angular.z = self.last_cmd.angular.z

        # find the most dangerous pedestrian
        threat = None
        min_dist = float('inf')

        for p in self.pedestrians:
            px, py = p['x'], p['y']
            vx, vy = p['vx'], p['vy']
            dist = math.hypot(px, py)

            # skip if inside robot or too far
            if dist < self.robot_radius or dist > self.watch_dist:
                continue

            # pedestrian angle (0° = ahead, 90° = side, 180° = behind)
            angle = abs(math.degrees(math.atan2(py, px)))

            # time to closest approach
            t_closest = self.time_to_closest_point(px, py, vx, vy)

            # skip departing pedestrians (already passed) unless very close
            if t_closest < 0 and dist > self.backup_dist:
                continue

            # skip pedestrians behind that are moving away
            if angle > 120:
                # check if approaching
                rel_vx = vx - self.robot_vx
                approaching = (px * rel_vx + py * vy) < -0.01
                if not approaching:
                    continue

            # calculate perpendicular distance to trajectory
            perp_dist = self.perp_dist_to_trajectory(px, py, vx, vy)

            # select closest threat
            if dist < min_dist:
                min_dist = dist
                threat = {
                    'px': px, 'py': py, 'vx': vx, 'vy': vy,
                    'dist': dist, 'angle': angle, 'perp_dist': perp_dist,
                    't_closest': t_closest
                }

        # no threat - pass through nav2 command
        if threat is None:
            if self.evading and (now - self.evade_start_time) > 1.0:
                self.evading = False
            
            self.backup_start_time = 0.0

            # reset dodge direction after extended period of no use
            if self.dodge_direction != 0 and (now - self.dodge_commit_time) > 5.0:
                self.dodge_direction = 0.0

            self.last_threat_dist = None
            self.current_action = None
            self.was_aimed = False

            # reset smoothing to nav2 command
            self.smooth_vx = cmd.linear.x
            self.smooth_wz = cmd.angular.z
            self.cmd_pub.publish(cmd)
            return

        # extract threat info
        px, py = threat['px'], threat['py']
        vx, vy = threat['vx'], threat['vy']
        dist = threat['dist']
        angle = threat['angle']
        perp_dist = threat['perp_dist']
        t_closest = threat['t_closest']

        # smooth threat distance to handle tracker noise/ID switching
        if self.last_threat_dist is not None and (now - self.last_threat_time) < 0.2:
            # if distance jumped > 0.3m in 200ms, likely noise - use average
            if abs(dist - self.last_threat_dist) > 0.3:
                dist = (dist + self.last_threat_dist) / 2
        self.last_threat_dist = dist
        self.last_threat_time = now

        # determine if robot is in pedestrian's path
        # use hysteresis: enter evade at 0.30m, exit at 0.50m
        if self.evading:
            robot_in_path = perp_dist < self.safe_threshold
        else:
            robot_in_path = perp_dist < self.in_path_threshold

        # --- priority 1: emergency - collision imminent (dist < 0.45m) ---
        if dist < self.emergency_dist:

            if not self.evading or (now - self.evade_start_time) > 2.0:
                self.evading = True
                self.evade_start_time = now
                self.evade_direction = self.compute_escape_steer(px, py, vx, vy)

            aimed = self.is_aimed_at_robot(px, py, vx, vy)

            # check backup timeout first
            if self.backup_start_time > 0 and (now - self.backup_start_time) > self.backup_timeout:
                # timeout - force forward to satisfy progress checker
                cmd.linear.x = self.forward_vel * 0.8
                cmd.angular.z = self.get_dodge_steer(py, now) * 0.8
                self.backup_start_time = 0.0
                self.get_logger().error(
                    f'EMERGENCY-FWD(timeout): d={dist:.2f} angle={angle:.0f}°'
                )
            elif angle < 60 and aimed:
                # ped ahead AND aimed at robot - backup
                cmd.linear.x = self.backup_vel
                cmd.angular.z = self.get_dodge_steer(py, now) * 0.5
                if self.backup_start_time == 0:
                    self.backup_start_time = now
                self.get_logger().error(
                    f'EMERGENCY-BACK: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}° aimed={aimed}'
                )
            elif angle < 60 and not aimed:
                # ped ahead but NOT aimed (crossing perpendicular) - YIELD
                cmd.linear.x = self.creep_vel
                cmd.angular.z = 0.0
                self.backup_start_time = 0.0
                self.get_logger().error(
                    f'EMERGENCY-YIELD: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}° (crossing)'
                )
            else:
                # ped to side/behind - go forward fast
                cmd.linear.x = self.forward_vel
                cmd.angular.z = self.evade_direction * 0.5 
                self.backup_start_time = 0.0
                self.get_logger().error(
                    f'EMERGENCY-FWD: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                )

        # --- priority 2: critical - very close (dist < 0.60m) ---
        elif dist < self.critical_dist:
            aimed = self.is_aimed_at_robot(px, py, vx, vy)

            if robot_in_path:
                # in path at close range - evade regardless of t_closest
                # only recompute evade_direction if not evading or if stale
                if not self.evading or (now - self.evade_start_time) > 2.0:
                    self.evading = True
                    self.evade_start_time = now
                    self.evade_direction = self.compute_escape_steer(px, py, vx, vy)

                if angle > 90:
                    cmd.linear.x = self.forward_vel
                    cmd.angular.z = self.evade_direction * 0.4
                    self.backup_start_time = 0.0
                    self.get_logger().warn(
                        f'CRITICAL-FWD(behind): d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                    )
                elif angle > 40:
                    cmd.linear.x = self.forward_vel
                    cmd.angular.z = self.evade_direction * 0.5  
                    self.backup_start_time = 0.0
                    self.get_logger().warn(
                        f'CRITICAL-FWD(side): d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                    )
                elif aimed:
                    # head-on AND aimed - backup with timeout check
                    if self.backup_start_time > 0 and (now - self.backup_start_time) > self.backup_timeout:
                        cmd.linear.x = self.forward_vel * 0.8
                        cmd.angular.z = self.get_dodge_steer(py, now) * 0.8
                        self.backup_start_time = 0.0
                        self.get_logger().warn(
                            f'CRITICAL-FWD(timeout): d={dist:.2f} angle={angle:.0f}°'
                        )
                    else:
                        cmd.linear.x = self.backup_vel
                        cmd.angular.z = self.get_dodge_steer(py, now) * 0.5
                        if self.backup_start_time == 0:
                            self.backup_start_time = now
                        self.get_logger().warn(
                            f'CRITICAL-BACK: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                        )
                else:
                    # head-on but NOT aimed - YIELD instead of backup
                    cmd.linear.x = self.creep_vel
                    cmd.angular.z = 0.0
                    self.backup_start_time = 0.0
                    self.get_logger().warn(
                        f'CRITICAL-YIELD: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}° (crossing)'
                    )
            else:
                # not in path but still close - slow significantly
                self.evading = False
                self.backup_start_time = 0.0
                cmd.linear.x = max(0.06, self.last_cmd.linear.x * 0.25)  # v8.5: raised min from 0.03
                cmd.angular.z = self.last_cmd.angular.z * 0.3
                self.get_logger().info(
                    f'CRITICAL-SLOW: d={dist:.2f} perp={perp_dist:.2f}'
                )

        # --- priority 3: caution - trajectory analysis ok at this range ---
        elif robot_in_path and t_closest > 0:
            # only recompute evade_direction if not evading or if stale
            if not self.evading or (now - self.evade_start_time) > 2.0:
                self.evading = True
                self.evade_start_time = now
                self.evade_direction = self.compute_escape_steer(px, py, vx, vy)

            aimed = self.is_aimed_at_robot(px, py, vx, vy)

            if angle > 90:
                cmd.linear.x = self.forward_vel
                cmd.angular.z = self.evade_direction * 0.4 
                self.backup_start_time = 0.0
                self.get_logger().info(
                    f'EVADE-FWD(behind): d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                )
            elif angle > 40:
                cmd.linear.x = self.forward_vel
                cmd.angular.z = self.evade_direction * 0.6 
                self.backup_start_time = 0.0
                self.get_logger().info(
                    f'EVADE-FWD(side): d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                )
            elif aimed and (dist < self.backup_dist or t_closest < 0.8):
            
                if self.backup_start_time > 0 and (now - self.backup_start_time) > self.backup_timeout:
                    cmd.linear.x = self.forward_vel * 0.8
                    cmd.angular.z = self.get_dodge_steer(py, now) * 0.8
                    self.backup_start_time = 0.0
                    self.get_logger().warn(
                        f'EVADE-FWD(timeout): d={dist:.2f} angle={angle:.0f}°'
                    )
                else:
                    cmd.linear.x = self.backup_vel
                    cmd.angular.z = self.get_dodge_steer(py, now) * 0.5
                    if self.backup_start_time == 0:
                        self.backup_start_time = now
                    self.get_logger().warn(
                        f'EVADE-BACK: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}°'
                    )
            elif not aimed:
                # not aimed - just yield, don't backup
                cmd.linear.x = self.creep_vel
                cmd.angular.z = 0.0
                self.backup_start_time = 0.0
                self.get_logger().info(
                    f'CAUTION-YIELD: d={dist:.2f} perp={perp_dist:.2f} angle={angle:.0f}° (crossing)'
                )
            else:
                cmd.linear.x = self.forward_vel
                cmd.angular.z = self.evade_direction * 0.6 
                self.backup_start_time = 0.0
                self.get_logger().info(
                    f'EVADE-FWD(ahead): d={dist:.2f} perp={perp_dist:.2f} t={t_closest:.1f}s'
                )

        # --- priority 4: not in path or ped departing -> creep/slow ---
        else:
            self.evading = False
            self.backup_start_time = 0.0

            if dist < 0.70:
                # creep instead of stop - satisfies progress checker
                cmd.linear.x = self.creep_vel
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f'CREEP: d={dist:.2f} perp={perp_dist:.2f} - letting ped pass'
                )
            elif dist < self.watch_dist:
                factor = max(0.2, (dist - 0.70) / 0.50)
                cmd.linear.x = max(0.06, self.last_cmd.linear.x * min(factor, 0.4))  # v8.5: raised min from 0.03
                self.get_logger().info(
                    f'SLOW: d={dist:.2f} perp={perp_dist:.2f}'
                )

        # determine action category from computed velocities
        if cmd.linear.x < 0:
            intended_action = 'BACKUP'
        elif cmd.linear.x >= self.forward_vel - 0.02:
            intended_action = 'EVADE'
        elif cmd.linear.x <= self.creep_vel + 0.02:
            intended_action = 'CREEP'
        else:
            intended_action = 'SLOW'

        # check if action switch is allowed
        if not self.set_action(intended_action, now):
            # switch denied - use previous safe velocities
            cmd.linear.x = self.last_safe_vx
            cmd.angular.z = self.last_safe_wz
        else:
            # switch allowed - save these as new safe velocities
            self.last_safe_vx = cmd.linear.x
            self.last_safe_wz = cmd.angular.z

        # clamp velocities
        cmd.linear.x = max(-0.15, min(0.18, cmd.linear.x))  # v8.5: reduced from 0.22
        cmd.angular.z = max(-1.2, min(1.2, cmd.angular.z))

        # apply smoothing to prevent jitter
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
