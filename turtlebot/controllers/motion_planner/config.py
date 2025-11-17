# motion planning config - robot specs and tunable parameters

# robot specs
WHEEL_RADIUS = 0.033  # meters
WHEEL_SEPARATION = 0.160  # meters
MAX_WHEEL_VELOCITY = 6.28  # rad/s
MAX_LINEAR_VELOCITY = 0.22  # m/s
MAX_ANGULAR_VELOCITY = 2.84  # rad/s
MAX_LINEAR_ACCELERATION = 1.0  # m/s²
MAX_ANGULAR_ACCELERATION = 3.0  # rad/s²

# webots sim params
TIME_STEP = 64  # ms
LEFT_MOTOR_NAME = "left wheel motor"
RIGHT_MOTOR_NAME = "right wheel motor"
LIDAR_NAME = "LDS-01"

# test sequence
TEST_FORWARD_DURATION = 1.0  # seconds
TEST_ROTATE_DURATION = 1.5
TEST_STOP_DURATION = 0.5
DEFAULT_FORWARD_SPEED = 2.0  # rad/s wheel velocity
DEFAULT_ROTATE_SPEED = 1.5  # rad/s

# odometry
ODOMETRY_UPDATE_RATE = TIME_STEP / 1000.0

# p-controller
K_LINEAR = 1.0
K_ANGULAR = 2.0
GOAL_TOLERANCE = 0.05  # meters

# waypoints
WAYPOINT_REACHED_THRESHOLD = 0.1  # meters

# dwa params
DWA_CONFIG = {
    "v_samples": 10,
    "omega_samples": 20,
    "sim_time": 2.0,
    "sim_dt": 0.1,
    "w_goal": 1.0,
    "w_heading": 0.5,
    "w_obstacle": 2.0,
    "w_smoothness": 0.3,
    "w_velocity": 0.2,
    "w_human": 3.0,
    "min_obstacle_distance": 0.3,
    "robot_radius": 0.105,
}
