import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
import uuid
import time

class KalmanTrack:
    """Represents a single tracked object."""
    def __init__(self, x, y):
        # Unique ID
        self.id = str(uuid.uuid4())[:8]

        # Initial timestamp
        self.last_time = time.time()

        # Initial state vector [x, y, vx, vy]
        self.x = np.array([[x], [y], [0.0], [0.0]])

        # State covariance
        self.P = np.eye(4) * 0.5

        # Process noise (tune later)
        self.Q = np.eye(4) * 0.05

        # Measurement matrix
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])

        # Measurement noise
        self.R = np.eye(2) * 0.1

    def predict(self, dt):
        """Kalman prediction step."""
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ])

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, meas_x, meas_y):
        """Kalman update step using centroid measurement."""
        z = np.array([[meas_x], [meas_y]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P


class KalmanTracker(Node):
    """Tracks multiple moving objects using Kalman Filters."""
    def __init__(self):
        super().__init__('kalman_tracker')

        self.subscription = self.create_subscription(
            MarkerArray,
            '/detected_humans',
            self.measurement_callback,
            10)

        self.publisher = self.create_publisher(
            MarkerArray,
            '/tracked_objects',
            10)

        self.tracks = []
        self.dist_threshold = 0.6   # cluster-to-track distance threshold
        self.get_logger().info("Kalman tracker started.")

    def measurement_callback(self, markers_msg):
        """Receives centroid measurements from detector."""
        measurements = [(m.pose.position.x, m.pose.position.y) 
                        for m in markers_msg.markers]

        if len(measurements) == 0:
            return

        # Update all tracks
        now = time.time()
        for track in self.tracks:
            dt = now - track.last_time
            track.last_time = now
            track.predict(dt)

        # Assign measurements to tracks
        used = set()
        for mx, my in measurements:
            matched = False
            for track in self.tracks:
                tx, ty = track.x[0,0], track.x[1,0]
                dist = np.hypot(mx - tx, my - ty)

                if dist < self.dist_threshold and track not in used:
                    track.update(mx, my)
                    used.add(track)
                    matched = True
                    break

            # No matching track → create new one
            if not matched:
                self.tracks.append(KalmanTrack(mx, my))

        # Remove stale tracks
        self.tracks = [t for t in self.tracks if np.hypot(t.x[2,0], t.x[3,0]) < 5.0]

        # Publish visualization
        self.publish_tracks()

    def publish_tracks(self):
        msg = MarkerArray()

        for t in self.tracks:
            # Current position
            cx, cy = t.x[0,0], t.x[1,0]
            vx, vy = t.x[2,0], t.x[3,0]

            # Marker: sphere for object
            m = Marker()
            m.header.frame_id = "lidar_link"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "tracked_objects"
            m.id = int(t.id[:4], 16)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.0
            m.scale.x = 0.25
            m.scale.y = 0.25
            m.scale.z = 0.25
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            msg.markers.append(m)

            # Marker: arrow for velocity
            v = Marker()
            v.header.frame_id = "lidar_link"
            v.header.stamp = m.header.stamp
            v.ns = "velocity"
            v.id = m.id + 10000
            v.type = Marker.ARROW
            v.action = Marker.ADD
            v.scale.x = 0.05   # shaft diameter
            v.scale.y = 0.1    # head diameter
            v.scale.z = 0.0
            v.color.a = 1.0
            v.color.r = 0.0
            v.color.g = 0.0
            v.color.b = 1.0
            v.points = [
                Point(x=cx, y=cy, z=0.0),
                Point(x=cx + vx, y=cy + vy, z=0.0)
            ]
            msg.markers.append(v)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KalmanTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
