import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
import time


# single object track with kalman filter
class KalmanTrack:
    _next_id = 0

    def __init__(self, x, y):
        self.id = KalmanTrack._next_id
        KalmanTrack._next_id += 1

        self.last_update_time = time.time()
        self.hits = 1  # number of successful associations
        self.misses = 0  # consecutive misses

        # state: [x, y, vx, vy]
        self.x = np.array([[x], [y], [0.0], [0.0]])

        # covariance
        self.P = np.eye(4) * 0.5

        # process noise - tuned for pedestrian motion
        self.Q = np.diag([0.01, 0.01, 0.1, 0.1])

        # measurement matrix
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])

        # measurement noise
        self.R = np.eye(2) * 0.05

    # predict next state
    def predict(self, dt):
        dt = min(dt, 0.5)  # cap dt to avoid explosion
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    # update with measurement
    def update(self, mx, my):
        z = np.array([[mx], [my]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.last_update_time = time.time()
        self.hits += 1
        self.misses = 0

    # track wasn't associated this frame
    def mark_missed(self):
        self.misses += 1

    @property
    def pos(self):
        return self.x[0, 0], self.x[1, 0]

    @property
    def vel(self):
        return self.x[2, 0], self.x[3, 0]

    @property
    def speed(self):
        return np.hypot(self.x[2, 0], self.x[3, 0])


# multi-object tracker using kalman filters
class KalmanTracker(Node):
    def __init__(self):
        super().__init__('kalman_tracker')

        self.subscription = self.create_subscription(
            MarkerArray, '/detected_humans', self.measurement_callback, 10)
        self.publisher = self.create_publisher(
            MarkerArray, '/tracked_objects', 10)

        self.tracks = []

        # tracking parameters
        self.assoc_dist = 0.5       # max distance for association
        self.max_misses = 5         # remove after this many misses
        self.min_hits = 2           # need this many hits to publish
        self.max_age = 2.0          # remove tracks older than this (seconds)
        self.min_speed = 0.12       # filter out very slow/stationary objects

        self.get_logger().info("kalman tracker started")

    def measurement_callback(self, msg):
        now = time.time()

        # extract measurements
        measurements = [(m.pose.position.x, m.pose.position.y)
                        for m in msg.markers if m.ns == 'humans']

        if not measurements and not self.tracks:
            return

        # predict all tracks
        for track in self.tracks:
            dt = now - track.last_update_time
            track.predict(dt)

        # associate measurements to tracks using greedy assignment
        used_tracks = set()
        used_meas = set()

        # sort by distance for greedy assignment
        associations = []
        for mi, (mx, my) in enumerate(measurements):
            for ti, track in enumerate(self.tracks):
                tx, ty = track.pos
                dist = np.hypot(mx - tx, my - ty)
                if dist < self.assoc_dist:
                    associations.append((dist, mi, ti))

        associations.sort()  # closest first

        for dist, mi, ti in associations:
            if mi in used_meas or ti in used_tracks:
                continue
            mx, my = measurements[mi]
            self.tracks[ti].update(mx, my)
            used_tracks.add(ti)
            used_meas.add(mi)

        # mark missed tracks
        for ti, track in enumerate(self.tracks):
            if ti not in used_tracks:
                track.mark_missed()

        # create new tracks for unassociated measurements
        for mi, (mx, my) in enumerate(measurements):
            if mi not in used_meas:
                self.tracks.append(KalmanTrack(mx, my))

        # remove stale tracks
        self.tracks = [t for t in self.tracks
                       if t.misses < self.max_misses
                       and (now - t.last_update_time) < self.max_age]

        # publish
        self.publish_tracks()

    def publish_tracks(self):
        msg = MarkerArray()

        for track in self.tracks:
            # only publish confirmed moving tracks
            if track.hits < self.min_hits:
                continue
            if track.speed < self.min_speed:
                continue

            cx, cy = track.pos
            vx, vy = track.vel

            # position marker
            m = Marker()
            m.header.frame_id = "lidar_link"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "tracked_objects"
            m.id = track.id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = 200_000_000
            msg.markers.append(m)

            # velocity arrow
            v = Marker()
            v.header = m.header
            v.ns = "velocity"
            v.id = track.id + 10000
            v.type = Marker.ARROW
            v.action = Marker.ADD
            v.scale.x = 0.03
            v.scale.y = 0.06
            v.scale.z = 0.0
            v.color.a = 1.0
            v.color.r = 0.0
            v.color.g = 0.0
            v.color.b = 1.0
            v.lifetime.sec = 0
            v.lifetime.nanosec = 200_000_000
            v.points = [
                Point(x=cx, y=cy, z=0.0),
                Point(x=cx + vx, y=cy + vy, z=0.0)
            ]
            msg.markers.append(v)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KalmanTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
