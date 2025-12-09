# scan filter - removes MOVING pedestrians from lidar scan for AMCL
# uses tracked_objects (confirmed moving objects from kalman tracker)
#
# CRITICAL: only filter confirmed MOVING objects, NOT static obstacles
# static obstacles (walls, boxes) must remain visible to AMCL for localization
# no startup fallback - ensures AMCL always has full static environment

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
import math
import time


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # tracked_objects = MOVING objects only (from kalman tracker)
        self.tracks_sub = self.create_subscription(
            MarkerArray, '/tracked_objects', self.tracks_callback, 10)

        self.filtered_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

        # tracked moving objects [(x, y), ...]
        self.tracked_objects = []
        self.last_track_time = 0.0
        self.track_timeout = 1.0

        # filter radius - just enough to cover pedestrian body
        self.filter_radius = 0.30  # reduced to be more precise

        # NO startup fallback - only filter confirmed MOVING objects
        # this ensures AMCL always sees static obstacles for localization

        self.log_counter = 0

        self.get_logger().info(
            'Scan filter: filtering MOVING objects only (tracked_objects), '
            f'radius={self.filter_radius}m'
        )

    def tracks_callback(self, msg: MarkerArray):
        """Receive tracked MOVING objects from kalman tracker."""
        self.tracked_objects = []
        for marker in msg.markers:
            if marker.ns == 'tracked_objects':
                x = marker.pose.position.x
                y = marker.pose.position.y
                self.tracked_objects.append((x, y))
        self.last_track_time = time.time()

    def scan_callback(self, msg: LaserScan):
        now = time.time()
        track_age = now - self.last_track_time

        # only filter if we have fresh tracked MOVING objects
        # no startup fallback - AMCL must always see static obstacles
        if track_age > self.track_timeout or not self.tracked_objects:
            self.filtered_pub.publish(msg)
            return

        # create filtered scan
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        filtered_ranges = list(msg.ranges)
        filtered_count = 0

        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max or math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # check if near any tracked moving object
            for ox, oy in self.tracked_objects:
                if math.hypot(x - ox, y - oy) < self.filter_radius:
                    filtered_ranges[i] = float('inf')
                    filtered_count += 1
                    break

        filtered.ranges = filtered_ranges
        filtered.intensities = msg.intensities if msg.intensities else []

        self.filtered_pub.publish(filtered)

        # periodic logging
        self.log_counter += 1
        if filtered_count > 0 and self.log_counter % 100 == 0:
            self.get_logger().info(
                f'Filtered {filtered_count} pts from {len(self.tracked_objects)} moving objects'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
