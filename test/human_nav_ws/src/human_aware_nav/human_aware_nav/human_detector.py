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
        
        # 聚类阈值
        self.cluster_threshold = 0.5
        self.get_logger().info("行人检测节点已启动！(Waiting for scan data...)")

    def scan_callback(self, msg):
        clusters = []
        current_cluster = []
        
        angle = msg.angle_min
        prev_x = 0
        prev_y = 0
        first_point = True

        for r in msg.ranges:
            if r < msg.range_min or r > msg.range_max or math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            if first_point:
                current_cluster.append((x, y))
                first_point = False
            else:
                dist = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                if dist < self.cluster_threshold:
                    current_cluster.append((x, y))
                else:
                    if len(current_cluster) > 3:
                        clusters.append(current_cluster)
                    current_cluster = [(x, y)]
            
            prev_x = x
            prev_y = y
            angle += msg.angle_increment
        
        if len(current_cluster) > 3:
            clusters.append(current_cluster)

        # 关键修复：把 scan 的时间戳传给 marker
        self.publish_markers(clusters, msg.header.stamp)

    def publish_markers(self, clusters, timestamp):
        marker_array = MarkerArray()
        
        for i, cluster in enumerate(clusters):
            sum_x = sum(p[0] for p in cluster)
            sum_y = sum(p[1] for p in cluster)
            center_x = sum_x / len(cluster)
            center_y = sum_y / len(cluster)
            
            marker = Marker()
            marker.header.frame_id = "lidar_link"
            # 关键修复：使用雷达数据的时间戳，保证 TF 同步
            marker.header.stamp = timestamp 
            
            marker.ns = "humans"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            # 设置生命周期为 0.2 秒，超时自动消失（防止旧的影子残留）
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000 
            
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
