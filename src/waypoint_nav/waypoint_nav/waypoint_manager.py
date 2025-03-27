#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import os

from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        # Listen for clicked points and store them as waypoints
        self.waypoints = []
        self.waypoint_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.waypoint_callback,
            1
        )

        # Subscribe to amcl_pose to get the robot's current position
        self.robot = None
        self.robot_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.robot_callback,
            1
        )

        # Publish to goal_pose
        self.goal_index = 0
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            1
        )

        # Timer callback for goal management (10 Hz)
        self.timer = self.create_timer(0.1, self.goal_callback)

    def waypoint_callback(self, data):
        self.waypoints.append(data)
        self.get_logger().info('Added waypoint: {0}'.format(data))

        if len(self.waypoints) >= 3:
            self.get_logger().info('Received 3 waypoints, sending to path planner')

    def robot_callback(self, data):
        self.robot = data.pose.pose.position

    def goal_callback(self):

        if len(self.waypoints) >= 3:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            # Set the position from the waypoint (x and y) and assign z = 0.0
            goal.pose.position.x = self.waypoints[self.goal_index].point.x
            goal.pose.position.y = self.waypoints[self.goal_index].point.y
            goal.pose.position.z = 0.0
            # Set default orientation as a unit quaternion (no rotation)
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)

            # Check if the robot is close to the goal
            if self.robot is not None:
                dist = np.sqrt((self.robot.x - goal.pose.position.x)**2 + (self.robot.y - goal.pose.position.y)**2)
                if dist < 0.55:
                    self.get_logger().info('Goal {0} reached!'.format(self.goal_index))
                    if self.goal_index < len(self.waypoints) - 1:
                        self.goal_index += 1
                    else:
                        self.get_logger().info('All goals reached!')
                        self.waypoints = []
                        self.goal_index = 0

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()