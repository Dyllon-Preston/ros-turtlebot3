#!/usr/bin/env python3
"""
go_to_goal.py:
This node implements a simple go-to-goal controller.

Subscribers:
- /odom (nav_msgs/Odometry): Provides the robot's current position and orientation.
- /current_goal (std_msgs/Float64MultiArray): Contains the goal information as [x, y, z, distTol, waitTime].

Publishers:
- /cmd_vel (geometry_msgs/Twist): Sends velocity commands to drive the robot.

The node computes the difference between the current position and the goal, calculates the desired heading,
and uses a proportional controller for angular velocity. It commands a constant forward speed until the robot 
is within the specified tolerance of the goal, at which point it stops.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Float64MultiArray, '/current_goal', self.goal_callback, 10)
        
        # Publisher for velocity commands
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Internal state
        self.current_pose = None   # np.array([x, y])
        self.current_yaw = None    # orientation (radians)
        self.goal = None           # Dictionary: {'x': float, 'y': float, 'z': float, 'distTol': float, 'waitTime': float}
        
        # Control gains and parameters
        self.k = 1
        self.l = 0.03 # 3 cm
        
    def goal_callback(self, msg: Float64MultiArray):
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

    def odom_callback(self, msg):
        self.robot = {
            'x': msg.x,
            'y': msg.y,
            'theta': msg.z
        }

    def control_loop(self):
        """
        Timer callback that computes the command to drive the robot toward the goal.
        """

        x_r = self.robot['x']
        y_r = self.robot['y']
        theta_r = self.robot['theta']

        x_g = self.goal['x']
        y_g = self.goal['y']

        l = self.l
        k = self.k

        x_p = x_r + l*np.cos(theta_r)
        y_p = y_r + l*np.sin(theta_r)

        e_x = x_g - x_p
        e_y = y_g - y_p

        v = k*(np.cos(theta_r) * e_x + np.sin(theta_r) * e_y)
        w = k/l*(-np.sin(theta_r) * e_x + np.cos(theta_r) * e_y)
        
        # Scale v and w proportionally to each other so they respect limits
        v_max = 0.2 # m/s
        w_max = 1.5 # rad/s
        if abs(v) > v_max or abs(w) > w_max:
            # Determine which limit we're hitting
            v_scale = abs(v) / v_max if abs(v) > v_max else 1.0
            w_scale = abs(w) / w_max if abs(w) > w_max else 1.0
            
            # Use the largest scaling factor to scale both v and w
            scale = max(v_scale, w_scale)
            v = v / scale
            w = w / scale

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
