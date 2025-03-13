
import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float32MultiArray

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/actual_odom', self.odom_callback, 1)
        self.obs_sub = self.create_subscription(Float32MultiArray, '/obstacle_vectors', self.obstacle_callback, 10)

        # Publisher for target point
        self.target_pub = self.create_publisher(Float32MultiArray, '/x_target', 10)
        
        # Timer callback for decision-making (10 Hz)
        self.timer = self.create_timer(0.1, self.update_state)

        # Internal state
        self.obstacle_vectors = None  # numpy array of shape (N, 3) or None
        self.robot = None

        # Thresholds and gains
        self.OBSTACLE_DISTANCE_LIMIT = 1.0  # meters: consider obstacles within this distance
        self.OBSTALCE_DISTANCE_THRESHOLD = 0.5 # meters
        self.DANGER_DISTANCE = 0.5              # meters: obstacle is dangerously close
        self.DIRECT_PATH_ANGLE_THRESHOLD = 0.3  # radians (~17°) tolerance for direct blockage
        self.GOAL_TOLERANCE = 0.1               # meters: goal reached if within this radius

        self.DANGER_DISTANCE = 0.5 # meters: obstacle is dangerously close (avoid obstacle)

    def obstacle_callback(self, msg: Float32MultiArray):
        """
        Callback to update the list of obstacle vectors.
        Expects a flattened array: [x1, y1, z1, x2, y2, z2, ...].
        """
        data = msg.data
        if len(data) % 3 != 0:
            self.get_logger().error("Obstacle message does not contain valid data.")
            return
        self.obstacle_vectors = np.array(data).reshape(-1, 3)

    def odom_callback(self, msg: Odometry):
        """
        Callback to update the robot's current pose and orientation from /odom.
        """
        self.robot = {
            'x': msg.x,
            'y': msg.y,
            'theta': msg.z
        }
    
    def update_state(self):
        """
        Decision-making logic to navigate to the goal while avoiding obstacles.

        Possible States:
        - Direct Path: No obstacles in the way --> Move directly to goal
        - Obstacle Detected: Obstacle within threshold distance --> Rotate to avoid
        - Follow Wall: Obstacle within threshold distance and angle --> Follow the wall
        - Danger Zone: Obstacle dangerously close --> Rotate and move directly backwards from obstacle
        - Goal Reached: Within goal tolerance --> Stop and publish goal reached message

        """

        if self.current_pose is None or self.obstacle_vectors is None:
            return

        robot_pos = np.array([self.robot['x'], self.robot['y']])

        # Compute the goal direction vector
        goal_direction = self.goal - robot_pos
        goal_distance = np.linalg.norm(goal_direction)
        goal_direction /= goal_distance

        # Get the nearest obstacle
        obstacle = self.obstacle_vectors[:3]
        obstacle_direction = obstacle[:2] - robot_pos
        obstacle_distance = np.linalg.norm(obstacle_direction)
        obstacle_direction /= obstacle_distance

        # Check for direct path to goal
        if obstacle_distance > self.OBSTACLE_DISTANCE_LIMIT:
            self.get_logger().info("Direct Path: No obstacles in the way.")
            self.publish_direction(goal_direction)
            return
        
        # Check for danger zone
        if obstacle_distance < self.DANGER_DISTANCE:
            self.get_logger().info("Danger Zone: Obstacle dangerously close.")
            self.publish_direction(-obstacle_direction)
            return
        
        # If we are within range of object, set target point to point tangent to circle centered on object
        if obstacle_distance < self.OBSTACLE_DISTANCE_TRESHOLD:
            self.get_logger().info("Following Wall: Moving tangentially around obstacle.")      
            # Calculate tangential direction (perpendicular to obstacle direction)
            # Use cross product with z-axis to get perpendicular vector
            tangent_direction = np.array([-obstacle_direction[1], obstacle_direction[0]])
                    
            # Choose the tangent that leads toward the goal
            # If dot product with goal is negative, use the other tangent
            if np.dot(tangent_direction, goal_direction) < 0:
                tangent_direction = -tangent_direction
                    
            # Publish the tangential direction to follow the wall
            self.publish_direction(tangent_direction)
            

