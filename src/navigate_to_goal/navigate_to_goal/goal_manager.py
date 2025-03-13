#!/usr/bin/env python3
"""
Goal Manager Node (goal_manager.py):
Reads in goal locations from a text file, publishes the current target goal,
and indicates success when all goals have been reached.

The goal file is expected to have the following format:
    X,  Y,  Z,  distTol,  waitTime
    x1, y1, z1, tol1, t1
    x2, y2, z2, tol2, t2
    x3, y3, z3, tol3, t3

Notes:
- The tolerance (tol) is assumed to be provided in centimeters and is converted to meters.
- The robot must remain within the tolerance for 10 seconds to consider the goal as reached.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import os

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String

class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        
        # Load goal file (assumed to be in the package directory or provided via parameter)
        print(os.getcwd())
        goal_file = os.path.join(os.getcwd(), 'src/navigate_to_goal/navigate_to_goal/wayPoints.txt')
        try:
            # Read and parse the goal file manually
            with open(goal_file, 'r') as f:
                lines = f.readlines()
            
            # Skip the header row and parse the data
            data = []
            for line in lines[1:]:  # Skip header row
                if line.strip():  # Skip empty lines
                    values = [float(val.strip()) for val in line.split(',')]
                    if len(values) == 5:
                        data.append(values)
            
            self.goals = np.array(data)  # shape (N, 5): [x, y, z, distTol, waitTime]
            self.get_logger().info(f"Loaded {len(data)} goals from {goal_file}.")
        except Exception as e:
            self.get_logger().error(f"Failed to load goal file: {e}")
            self.goals = np.empty((0, 5))
        
        # Index for the current goal
        self.current_goal_index = 0
        self.wait_start_time = None  # Timestamp when the robot first enters goal tolerance
        
        # Subscriber: listen to odometry to get robot's current position
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_pose = None  # Will be set to [x, y]
        
        # Publishers:
        # Publish current goal (as a flattened list: [x, y, z, distTol, waitTime])
        self.goal_pub = self.create_publisher(Float32MultiArray, '/current_goal', 10)
        # Publish goal status (e.g., "GOALS_COMPLETE")
        self.status_pub = self.create_publisher(String, '/goal_status', 10)
        
        # Timer callback for state management (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def odom_callback(self, msg: Odometry):
        """
        Update the current robot pose from odometry.
        """
        pos = msg.pose.pose.position
        self.current_pose = np.array([pos.x, pos.y])
        
    def timer_callback(self):
        """
        Timer callback to publish the current goal and check for goal completion.
        """
        # If all goals have been processed, publish success status
        if self.current_goal_index >= len(self.goals):
            success_msg = String()
            success_msg.data = "GOALS_COMPLETE"
            self.status_pub.publish(success_msg)
            self.get_logger().info("All goals completed!")
            # Optionally, clear the current goal if desired:
            empty_goal = Float32MultiArray()
            empty_goal.data = []
            self.goal_pub.publish(empty_goal)
            return
        
        # Get current goal parameters
        current_goal = self.goals[self.current_goal_index]  # [x, y, z, distTol, waitTime]
        goal_x, goal_y, goal_z, distTol, waitTime = current_goal
        
        # Publish the current goal as a flattened list of floats
        goal_msg = Float32MultiArray()
        goal_msg.data = [goal_x, goal_y, goal_z, distTol, waitTime]
        self.goal_pub.publish(goal_msg)
        
        # Check if robot's pose is available to evaluate progress
        if self.current_pose is None:
            return
        
        # Compute the distance from the robot to the current goal (using x,y coordinates)
        goal_position = np.array([goal_x, goal_y])
        distance_to_goal = np.linalg.norm(goal_position - self.current_pose)
        
        # If the robot is within the tolerance, start (or continue) the waiting timer
        if distance_to_goal < distTol:
            if self.wait_start_time is None:
                self.wait_start_time = self.get_clock().now()
                self.get_logger().info(f"Entered goal region (Goal {self.current_goal_index}) - starting wait timer.")
            else:
                # Calculate elapsed time in seconds
                elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds * 1e-9
                if elapsed >= waitTime:
                    self.get_logger().info(f"Goal {self.current_goal_index} completed after waiting {waitTime} seconds.")
                    # Advance to the next goal and reset the wait timer
                    self.current_goal_index += 1
                    self.wait_start_time = None
        else:
            # Not yet within the tolerance; reset waiting timer if it was previously set
            if self.wait_start_time is not None:
                self.get_logger().info(f"Left goal region (Goal {self.current_goal_index}); resetting wait timer.")
            self.wait_start_time = None

def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()