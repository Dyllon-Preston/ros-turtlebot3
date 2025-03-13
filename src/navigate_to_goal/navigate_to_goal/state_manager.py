
import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.obs_sub = self.create_subscription(Float32MultiArray, '/obstacle_vectors', self.obstacle_callback, 10)

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer callback for decision-making (10 Hz)
        self.timer = self.create_timer(0.1, self.update_state)

        # Internal state
        self.current_pose = None   # (x, y)
        self.current_yaw = None    # orientation (radians)
        self.obstacle_vectors = None  # numpy array of shape (N, 3) or None

        # Set a fixed goal in global coordinates (can be parameterized or loaded from file)
        self.goal = np.array([2.0, 0.0])  # Example: goal at (2.0, 0.0)

        # Thresholds and gains
        self.OBSTACLE_DISTANCE_THRESHOLD = 1.0  # meters: consider obstacles within this distance
        self.DANGER_DISTANCE = 0.5              # meters: obstacle is dangerously close
        self.DIRECT_PATH_ANGLE_THRESHOLD = 0.3  # radians (~17°) tolerance for direct blockage
        self.GOAL_TOLERANCE = 0.1               # meters: goal reached if within this radius

        self.DANGER_DISTANCE = 0.5 # meters: obstacle is dangerously close (avoid obstacle)