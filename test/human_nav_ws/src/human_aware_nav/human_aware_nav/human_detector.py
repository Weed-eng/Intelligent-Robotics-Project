import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import math

class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector_node')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        self.marker_publisher = self.create_publisher(MarkerArray, '/detected_humans', 10)
        
        self.cluster_threshold = 0.25  # more realistic than 0.5
        self.get_logger().info("Human detector started – waiting for /scan...")

    def scan_callback(self, msg):
        clusters = []
        current_cluster = []

        angle = msg.angle_min
        prev_x = None
        prev_y = None

        for r in msg.ranges:
            if (r < msg.range_min or r > msg.range_max 
                or math.isinf(r) or math.isnan(r)):
                angle += msg.angle_increment
                prev_x, prev_y = None, None
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            if prev_x is None:
                current_cluster.append((x, y))
            else:
                dist = math.hypot(x - prev_x, y - prev_y)
                if dist < self.cluster_threshold:
                    current_cluster.append((x, y))
                else:
                    if len(current_cluster) >= 3:
                        clusters.append(current_cluster)
                    current_cluster = [(x, y)]

            prev_x, prev_y = x, y
            angle += msg.angle_increment
        
        if len(current_cluster) >= 3:
            clusters.append(current_cluster)

        self.publish_markers(clusters, msg.header.stamp)

    def publish_markers(self, clusters, timestamp):
        marker_array = MarkerArray()
        
        for i, cluster in enumerate(clusters):
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)

            marker = Marker()
            marker.header.frame_id = "base_scan"  # IMPORTANT FIX
            marker.header.stamp = timestamp

            marker.ns = "humans"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = 0.0

            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 300_000_000  # 0.3 seconds
            
            marker_array.markers.append(marker)
            
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = HumanDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
