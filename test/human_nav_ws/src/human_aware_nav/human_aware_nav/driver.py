import rclpy
import math
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from webots_ros2_driver.webots_controller import WebotsController
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# turtlebot3 burger specs
WHEEL_RADIUS = 0.033  # meters
WHEEL_SEPARATION = 0.16  # meters

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

        # time bridging - map sim time to wall clock for slam_toolbox compatibility
        self.system_start_time = self.my_ros_node.get_clock().now()
        self.sim_start_time = self.__robot.getTime()

        self.my_ros_node.get_logger().info("Driver initialized with wheel encoder odometry")

        # get devices
        self.lidar = self.__robot.getDevice('Lidar')
        self.left_motor = self.__robot.getDevice('left wheel motor')
        self.right_motor = self.__robot.getDevice('right wheel motor')

        # get position sensors from motors for accurate odometry
        self.left_sensor = self.left_motor.getPositionSensor()
        self.right_sensor = self.right_motor.getPositionSensor()

        # enable position sensors
        if self.left_sensor:
            self.left_sensor.enable(self.__timestep)
        if self.right_sensor:
            self.right_sensor.enable(self.__timestep)

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

        # publish static TF once at startup (base_link -> lidar_link)
        self._publish_static_transforms()

        # odometry state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # previous wheel positions for encoder-based odometry
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.first_odom_update = True

    def _publish_static_transforms(self):
        # static transform: base_link -> lidar_link (published once, latched)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.my_ros_node.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'lidar_link'
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.12
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_tf)
        self.my_ros_node.get_logger().info("Published static TF: base_link -> lidar_link")

    def __cmd_vel_callback(self, twist):
        # convert cmd_vel to wheel velocities using differential drive kinematics
        linear_v = twist.linear.x
        angular_w = twist.angular.z

        MAX_WHEEL_VEL = 6.67  # TurtleBot3Burger motor limit (rad/s)

        left_vel = (linear_v - angular_w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        right_vel = (linear_v + angular_w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS

        # clamp to motor limits
        left_vel = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, left_vel))
        right_vel = max(-MAX_WHEEL_VEL, min(MAX_WHEEL_VEL, right_vel))

        if self.left_motor and self.right_motor:
            self.left_motor.setVelocity(left_vel)
            self.right_motor.setVelocity(right_vel)

    def step(self):
        if hasattr(self, 'my_ros_node'):
            rclpy.spin_once(self.my_ros_node, timeout_sec=0)

        current_sim_time = self.__robot.getTime()

        # time bridging: map simulation time to wall clock
        elapsed_sim_seconds = current_sim_time - self.sim_start_time
        current_ros_time = self.system_start_time + Duration(seconds=elapsed_sim_seconds)
        ros_stamp = current_ros_time.to_msg()

        # --- ENCODER-BASED ODOMETRY (accurate, not command-based) ---
        # read actual wheel positions from encoders
        left_pos = self.left_sensor.getValue() if self.left_sensor else 0.0
        right_pos = self.right_sensor.getValue() if self.right_sensor else 0.0

        # on first update, just store positions
        if self.first_odom_update:
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            self.first_odom_update = False
        else:
            # calculate wheel travel distances
            delta_left = (left_pos - self.prev_left_pos) * WHEEL_RADIUS
            delta_right = (right_pos - self.prev_right_pos) * WHEEL_RADIUS

            # differential drive kinematics
            delta_center = (delta_left + delta_right) / 2.0
            delta_th = (delta_right - delta_left) / WHEEL_SEPARATION

            # update pose using mid-point approximation
            self.x += delta_center * math.cos(self.th + delta_th / 2.0)
            self.y += delta_center * math.sin(self.th + delta_th / 2.0)
            self.th += delta_th

            # normalize theta to [-pi, pi]
            self.th = math.atan2(math.sin(self.th), math.cos(self.th))

            # store for next iteration
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos

        # quaternion from theta
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        # publish TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = ros_stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # publish Odometry message
        odom = Odometry()
        odom.header.stamp = ros_stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_publisher.publish(odom)

        # publish LaserScan
        if self.lidar and self.scan_publisher:
            ranges = self.lidar.getRangeImage()
            if ranges:
                msg = LaserScan()
                msg.header.stamp = ros_stamp
                msg.header.frame_id = 'lidar_link'

                num_ranges = len(ranges)
                fov = self.lidar.getFov()

                # Webots LiDAR returns ranges from -FOV/2 to +FOV/2
                # BUT the spin direction might be opposite to ROS convention
                # Try reversing the ranges to fix mirrored scans
                ranges_list = list(ranges)
                ranges_list.reverse()  # flip scan direction

                msg.angle_min = -fov / 2
                msg.angle_max = fov / 2
                # use actual number of ranges, not getHorizontalResolution()
                msg.angle_increment = fov / num_ranges
                msg.range_min = self.lidar.getMinRange()
                msg.range_max = self.lidar.getMaxRange()
                msg.ranges = ranges_list
                self.scan_publisher.publish(msg)
