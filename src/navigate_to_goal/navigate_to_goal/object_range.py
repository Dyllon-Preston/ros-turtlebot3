#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT
)

def segment_obstacles(ranges, angles, range_min, range_max, diff_threshold, min_observations = 3):
    """
    Segments the LiDAR scan data into obstacles using DFS while skipping NaN values.
    
    Parameters:
        ranges (np.array): Array of range measurements.
        angles (np.array): Array of corresponding angles.
        range_min (float): Minimum valid range.
        range_max (float): Maximum valid range.
        diff_threshold (float): Maximum allowed difference between consecutive readings 
                                to consider them part of the same obstacle.
        min_observations (int): Minimum number of readings required to classify a segment as an obstacle.
                                
    Returns:
        valid_ranges (list): List of minimum ranges for each detected obstacle.
        valid_angles (list): List of angles corresponding to the minimum range for each obstacle.
    """
    n = len(ranges)
    visited = [False] * n
    segments = []
    
    def dfs(idx, segment):
        visited[idx] = True
        segment.append(idx)
        # Check neighbors in the circular array (previous and next)
        for neighbor in [(idx - 1) % n, (idx + 1) % n]:
            if visited[neighbor]:
                continue
            # Skip if neighbor reading is NaN
            if np.isnan(ranges[neighbor]):
                visited[neighbor] = True
                continue
            # Only consider valid measurements
            if ranges[neighbor] < range_min or ranges[neighbor] > range_max:
                visited[neighbor] = True
                continue
            # Check if the difference is within threshold to be in the same segment
            if abs(ranges[idx] - ranges[neighbor]) < diff_threshold:
                dfs(neighbor, segment)
    
    # Iterate over all indices and use DFS to segment contiguous measurements
    for i in range(n):
        if visited[i]:
            continue
        if np.isnan(ranges[i]) or ranges[i] < range_min or ranges[i] > range_max:
            visited[i] = True
            continue
        segment = []
        dfs(i, segment)
        if len(segment) >= min_observations:  # Only keep valid segments
            segments.append(segment)
    
    # For each segment, select the measurement with the minimum range.
    valid_ranges = []
    valid_angles = []
    for seg in segments:
        # Choose the index with the minimum range from the segment
        min_idx = min(seg, key=lambda idx: ranges[idx])
        valid_ranges.append(ranges[min_idx])
        valid_angles.append(angles[min_idx])
    
    return valid_ranges, valid_angles

class ObjectRange(Node):

    """
    ROS2 Node for detecting objects using LiDAR data based on gaps in scan ranges.
    
    Subscribes to:
    - /scan (LaserScan): LiDAR scan data containing distance measurements.
    
    Publishes:
    - /obstacle_vectors (Float32MultiArray): List of detected object positions in [x, y, z] format.
    """

    def __init__(self):
        """
        Initializes the LidarProcessor node.
        """
        super().__init__('lidar_processor')

        # LiDAR Subscriber
        self.lidar_subscriber = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile
        )

        # Obstacle Vector Publisher
        self.obstacle_publisher = self.create_publisher(
            Float32MultiArray, '/obstacle_vectors', 1
        )

        self.diff_threshold = 0.15  # Minimum difference between ranges to detect a new obstacle
        self.observations_threshold = 5

    def lidar_callback(self, msg: LaserScan):
        """
        Callback function to process LiDAR data and publish an ordered list of obstacle vectors.
        """

        # Extract lidar parameters
        range_min = msg.range_min
        range_max = msg.range_max
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Compute angles and filter valid readings
        ranges = np.array(msg.ranges)
        angles = np.arange(angle_min, angle_min + len(ranges) * angle_increment, angle_increment)

        # Find obstacles based on gaps in the range data
        valid_ranges, valid_angles = segment_obstacles(ranges, angles, range_min, 1, self.diff_threshold, self.observations_threshold)
        valid_ranges = np.array(valid_ranges)
        valid_angles = np.array(valid_angles)
        valid_angles += np.pi/2

        if len(valid_ranges) == 0:
            self.get_logger().warn("No valid obstacles detected.")
            msg = Float32MultiArray()
            msg.data = []
            self.obstacle_publisher.publish(msg)
            return

        # Sort obstacles by distance
        sorted_indices = np.argsort(valid_ranges)
        sorted_ranges = valid_ranges[sorted_indices]
        sorted_angles = valid_angles[sorted_indices]

        # Flattened list of [x, y, z] values
        obstacle_vectors = []
        for r, theta in zip(sorted_ranges, sorted_angles):
            x = r * np.cos(theta)  # X in robot frame
            y = r * np.sin(theta)  # Y in robot frame
            z = 0.0  # No height information from 2D LiDAR
            obstacle_vectors.extend([x, y, z])  # Flatten into a list

        # Publish the ordered list of obstacle vectors as a Float32MultiArray
        msg = Float32MultiArray()
        msg.data = obstacle_vectors
        self.obstacle_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
