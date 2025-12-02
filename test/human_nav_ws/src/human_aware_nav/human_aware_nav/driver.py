import rclpy
import math
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from webots_ros2_driver.webots_controller import WebotsController
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class MyRobotDriver(WebotsController):
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception:
            pass
        self.my_ros_node = rclpy.create_node('my_robot_driver_node')
        
        # --- 时间桥接初始化 ---
        # 1. 获取启动时的系统真实时间
        self.system_start_time = self.my_ros_node.get_clock().now()
        # 2. 获取启动时的仿真时间
        self.sim_start_time = self.__robot.getTime()
        
        self.my_ros_node.get_logger().warn(f">>> [时间桥接已启动] 将仿真数据伪装为系统时间 <<<")

        # 获取设备
        self.lidar = self.__robot.getDevice('Lidar')
        self.left_motor = self.__robot.getDevice('left wheel motor')
        self.right_motor = self.__robot.getDevice('right wheel motor')

        if self.left_motor:
            self.left_motor.setPosition(float('inf'))
            self.left_motor.setVelocity(0)
        if self.right_motor:
            self.right_motor.setPosition(float('inf'))
            self.right_motor.setVelocity(0)
            
        if self.lidar:
            self.lidar.enable(self.__timestep)
            self.scan_publisher = self.my_ros_node.create_publisher(LaserScan, '/scan', 1)

        self.cmd_vel_subscriber = self.my_ros_node.create_subscription(
            Twist, '/cmd_vel', self.__cmd_vel_callback, 1
        )
        
        self.odom_publisher = self.my_ros_node.create_publisher(Odometry, '/odom', 1)
        self.tf_broadcaster = TransformBroadcaster(self.my_ros_node)
        self.static_broadcaster = StaticTransformBroadcaster(self.my_ros_node)
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.__robot.getTime()
        self.current_v = 0.0
        self.current_w = 0.0

    def __cmd_vel_callback(self, twist):
        self.current_v = twist.linear.x
        self.current_w = twist.angular.z
        
        L = 0.16
        R = 0.033
        
        left_vel = (self.current_v - self.current_w * L / 2) / R
        right_vel = (self.current_v + self.current_w * L / 2) / R

        if self.left_motor and self.right_motor:
            self.left_motor.setVelocity(left_vel)
            self.right_motor.setVelocity(right_vel)

    def step(self):
        if hasattr(self, 'my_ros_node'):
            rclpy.spin_once(self.my_ros_node, timeout_sec=0)

        current_sim_time = self.__robot.getTime()
        dt = current_sim_time - self.last_time
        self.last_time = current_sim_time

        # --- 时间桥接计算 (核心修复) ---
        # 计算仿真运行了多久
        elapsed_sim_seconds = current_sim_time - self.sim_start_time
        # 将这段时长加到系统启动时间上
        # 结果：数据看起来像是刚刚发生的，同时保持了仿真的连贯性
        current_ros_time = self.system_start_time + Duration(seconds=elapsed_sim_seconds)
        ros_stamp = current_ros_time.to_msg()

        # 运动学计算
        delta_x = (self.current_v * math.cos(self.th)) * dt
        delta_y = (self.current_v * math.sin(self.th)) * dt
        delta_th = self.current_w * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        # 发布 Static TF (base_link -> lidar_link)
        static_tf = TransformStamped()
        static_tf.header.stamp = ros_stamp
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'lidar_link'
        static_tf.transform.translation.z = 0.12
        static_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(static_tf)

        # 发布 Dynamic TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = ros_stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 发布 Odom
        odom = Odometry()
        odom.header.stamp = ros_stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_publisher.publish(odom)

        # 发布 Scan
        if self.lidar and self.scan_publisher:
            ranges = self.lidar.getRangeImage()
            if ranges:
                msg = LaserScan()
                msg.header.stamp = ros_stamp # 这里的 stamp 现在是真实系统时间了！
                msg.header.frame_id = 'lidar_link'
                msg.angle_min = -self.lidar.getFov() / 2
                msg.angle_max = self.lidar.getFov() / 2
                msg.angle_increment = self.lidar.getFov() / self.lidar.getHorizontalResolution()
                msg.range_min = self.lidar.getMinRange()
                msg.range_max = self.lidar.getMaxRange()
                msg.ranges = ranges
                self.scan_publisher.publish(msg)
