
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

# Define state constants
STATE_GO_TO_GOAL = 0
STATE_AVOID_OBSTACLE = 1
STATE_DETOUR = 2

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Point, '/actual_odom', self.odom_callback, 1)
        self.obs_sub = self.create_subscription(Float32MultiArray, '/obstacle_vectors', self.obstacle_callback, 1)
        self.goal_sub = self.create_subscription(Float32MultiArray, '/current_goal', self.goal_callback, 1)

        # Publisher for target point
        self.target_pub = self.create_publisher(Float32MultiArray, '/x_target', 1)
        
        # Timer callback for decision-making (10 Hz)
        self.timer = self.create_timer(0.1, self.update_state)

        self.state = None

        # Internal state
        self.obstacle_vectors = None  # numpy array of shape (N, 3) or None
        self.robot = None
        self.goal = None
        self.detour_point = None
        self.safe_point = None

        # Thresholds and gains
        self.DANGER_DISTANCE = 0.35 # meters: obstacle is dangerously close (avoid obstacle)
        self.OBSTACLE_DISTANCE_THRESHOLD = 0.4 # meters
        self.GOAL_TOLERANCE = 0.1               # meters: goal reached if within this radius
        self.OBSTACLE_RADIUS = 0.5



    def goal_callback(self, msg: Float32MultiArray):
        """
        Callback to update the current goal from /current_goal.
        Expects a flattened array: [x, y, z, distTol, waitTime].
        """
        data = msg.data
        self.goal = {
            'x': data[0],
            'y': data[1],
            'z': data[2],
            'distTol': data[3],
            'waitTime': data[4]
        }

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

    def odom_callback(self, msg: Point):
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

        if self.robot is None or self.obstacle_vectors is None or self.goal is None:
            return

        # Get robot and goal positions in global frame
        R = np.array([self.robot['x'], self.robot['y']])
        G = np.array([self.goal['x'], self.goal['y']])
        goal_distance = np.linalg.norm(G - R)

        # Determine obstacle information (if available)
        obstacle_distance = float('inf')
        O = None  # Global coordinates of the nearest obstacle
        if self.obstacle_vectors is not None and self.obstacle_vectors.shape[0] > 0:
            obs = self.obstacle_vectors[0]
            obstacle_rel = np.array(obs[:2])  # in robot frame
            # Convert obstacle from robot frame to global frame
            O = R + obstacle_rel
            obstacle_distance = np.linalg.norm(obstacle_rel)

        if self.state is STATE_DETOUR and np.linalg.norm(self.detour_point - R) < 0.07:
            self.state = STATE_GO_TO_GOAL
        
        if self.state is STATE_AVOID_OBSTACLE and obstacle_distance > self.OBSTACLE_DISTANCE_THRESHOLD:
            self.state = STATE_GO_TO_GOAL
            
        self.get_logger().info(f"Goal Distance {goal_distance} | Obstacle Distance {obstacle_distance} | Detour Point {self.detour_point}")

        # --- State Transitions ---
        if self.state is not STATE_DETOUR and obstacle_distance < self.DANGER_DISTANCE:
            self.state = STATE_AVOID_OBSTACLE
            obstacle_direction = (O - R)/np.linalg.norm(O - R)
            self.safe_point = R - 0.2*obstacle_direction
        elif self.state is not STATE_DETOUR and self.state is not STATE_AVOID_OBSTACLE and obstacle_distance < self.OBSTACLE_DISTANCE_THRESHOLD and O is not None:
            # Determine if the direct path from R to G is obstructed by the obstacle circle.
            RG = G - R
            RG_norm = RG / np.linalg.norm(RG)
            t = np.dot(O - R, RG_norm)
            P = R + t * RG_norm   # projection of O onto RG
            d = np.linalg.norm(O - P)
            if d < self.OBSTACLE_RADIUS:
                self.state = STATE_DETOUR
                self.detour_point = self.compute_detour_point(R, G, O, self.OBSTACLE_RADIUS)
            else:
                self.state = STATE_GO_TO_GOAL
        elif self.state is not STATE_DETOUR and self.state is not STATE_AVOID_OBSTACLE:
            self.state = STATE_GO_TO_GOAL
        

        self.get_logger().info(f"State: {self.state}")

        if self.state == STATE_GO_TO_GOAL:
            self.publish_point(G)
        elif self.state == STATE_DETOUR:
            self.publish_point(self.detour_point)
        elif self.state == STATE_AVOID_OBSTACLE:
            self.publish_point(self.safe_point)



    def publish_point(self, point):
        msg = Float32MultiArray()
        msg.data = list(point)
        self.target_pub.publish(msg)

    def compute_detour_point(self, R, G, O, r):
        """ Compute the detour point T on the circumference of a circular obstacle so that the robot can safely bypass the obstacle.
        
        The approach is:
        - Draw a line from R to G.
        - Compute the perpendicular line passing through O.
        - Find the two intersection points of this perpendicular with the circle.
        - Return the intersection point that is closest to R.
        
        Arguments:
            R: np.array([x_r, y_r]) - Robot position.
            G: np.array([x_g, y_g]) - Goal position.
            O: np.array([x_o, y_o]) - Obstacle center.
            r: float - Obstacle radius.
            
        Returns:
            np.array([x_t, y_t]) - The detour point on the circle.
        """
        # Compute the vector along R -> G and normalize
        RG = G - R
        RG_norm = RG / np.linalg.norm(RG)
        
        # Compute a perpendicular unit vector (rotate by 90°)
        perp = np.array([-RG_norm[1], RG_norm[0]])
        
        # Compute the two intersection points of the perpendicular line through O with the circle
        T1 = O + r * perp
        T2 = O - r * perp
        
        # Return the point that is closer to the goal
        if np.linalg.norm(G - T1) < np.linalg.norm(G - T2):
            return T1
        else:
            return T2


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()