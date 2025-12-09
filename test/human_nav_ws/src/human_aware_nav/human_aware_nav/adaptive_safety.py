# adaptive safety - predictive pedestrian avoidance
# complements DWB (reactive) with prediction using kalman velocities
# ONLY intervenes when there's a real collision threat
# lets DWB handle normal obstacle avoidance

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
        self.human_radius = 0.10
        self.safety_margin = 0.08
        self.collision_dist = self.robot_radius + self.human_radius + self.safety_margin  # ~0.30m

        self.robot_max_speed = 0.22
        self.robot_max_turn = 1.5

        # thresholds - less aggressive than before
        self.react_dist = 1.5        # only consider pedestrians within this range
        self.slow_dist = 0.8         # start slowing down
        self.stop_dist = 0.45        # stop if this close
        self.emergency_dist = 0.25   # emergency backup

        # steering state - commit to direction
        self.steer_direction = 0     # -1 = right, 0 = none, 1 = left
        self.steer_commit_time = 0.0
        self.steer_commit_duration = 1.0  # commit for 1 second

        self.get_logger().info(
            f'Adaptive safety: collision_dist={self.collision_dist:.2f}m, '
            f'react_dist={self.react_dist:.2f}m'
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

        # find the most threatening pedestrian
        closest_dist = float('inf')
        threat_ped = None
        threat_level = 'NONE'  # NONE, SLOW, STOP, EMERGENCY

        for p in self.pedestrians:
            px, py = p['x'], p['y']
            vx, vy = p['vx'], p['vy']
            dist = math.hypot(px, py)

            # skip pedestrians behind us or too far
            if px < -0.1 or dist > self.react_dist:
                continue

            # is this pedestrian approaching us?
            ped_speed = math.hypot(vx, vy)
            approach_speed = -(px * vx + py * vy) / dist if dist > 0.01 else 0

            # pedestrian walking toward robot?
            ped_toward_robot = approach_speed > 0.05

            # check if this is a real threat
            if dist < closest_dist:
                closest_dist = dist
                threat_ped = p

                if dist < self.emergency_dist:
                    threat_level = 'EMERGENCY'
                elif dist < self.stop_dist and (ped_toward_robot or ped_speed < 0.1):
                    threat_level = 'STOP'
                elif dist < self.slow_dist:
                    threat_level = 'SLOW'

        # handle threat
        if threat_level == 'EMERGENCY' and threat_ped:
            # emergency - pedestrian very close
            px, py = threat_ped['x'], threat_ped['y']

            if px > 0:
                # pedestrian in front - stop or backup
                cmd.linear.x = -0.05  # gentle backup
                cmd.angular.z = 1.0 if py > 0 else -1.0  # turn away
                self.get_logger().warn(f'EMERGENCY: ped at {closest_dist:.2f}m')
            else:
                # pedestrian behind - move forward
                cmd.linear.x = self.robot_max_speed
                cmd.angular.z = 0.5 if py > 0 else -0.5
                self.get_logger().warn(f'EMERGENCY ESCAPE: ped behind at {closest_dist:.2f}m')

        elif threat_level == 'STOP':
            cmd.linear.x = 0.0
            self.get_logger().info(f'STOP: ped at {closest_dist:.2f}m')

        elif threat_level == 'SLOW' and threat_ped:
            # slow down and steer around
            px, py = threat_ped['x'], threat_ped['y']
            vx, vy = threat_ped['vx'], threat_ped['vy']

            # determine steer direction - perpendicular to pedestrian's path
            # if ped moving right (vy < 0), go left (+)
            # if ped moving left (vy > 0), go right (-)
            # if ped stationary, go away from them
            if math.hypot(vx, vy) > 0.1:
                desired_dir = 1 if vy < 0 else -1
            else:
                desired_dir = -1 if py > 0 else 1

            # commit to steering direction
            if now - self.steer_commit_time > self.steer_commit_duration:
                self.steer_direction = desired_dir
                self.steer_commit_time = now

            # apply steering with committed direction
            steer_amount = self.steer_direction * 0.8  # moderate turn
            cmd.angular.z = cmd.angular.z * 0.3 + steer_amount * 0.7

            # slow down based on distance
            slow_factor = (closest_dist - self.stop_dist) / (self.slow_dist - self.stop_dist)
            slow_factor = max(0.2, min(1.0, slow_factor))
            cmd.linear.x = cmd.linear.x * slow_factor

            direction = 'LEFT' if self.steer_direction > 0 else 'RIGHT'
            self.get_logger().info(f'STEER {direction}: ped at {closest_dist:.2f}m')

        # clamp velocities
        cmd.linear.x = max(-0.1, min(self.robot_max_speed, cmd.linear.x))
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
