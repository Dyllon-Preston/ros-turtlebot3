#!/usr/bin/env python3

"""
ObjectRange Node: Computes the range to a detected object using lidar data and the object's angles.

Group Name: Levi
Group Members:
- Dyllon Preston
- Richi Dubey

This ROS2 node subscribes to the object's angles and lidar scan data, computes the range of the detected object,
and publishes the range along with a timestamp. For multiple angles, it collects the distance at each angle,
filters out outliers, and uses the mean of the remaining distances as the reported range.

Dependencies:
- rclpy: ROS2 Python client library
- sensor_msgs.msg: For receiving LaserScan data
- std_msgs.msg: For publishing Float32MultiArray (used to send range data)
- numpy: For mathematical operations (angle conversion and outlier filtering)
"""

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

class ObjectRange(Node):
    """
    ROS2 Node for computing the range to an object based on lidar data and object's angles.

    Subscribes to:
    - /object_angle (Float32MultiArray): The angles at which the object is detected.
    - /scan (LaserScan): Lidar scan data for distance measurements.

    Publishes:
    - object_range (Float32MultiArray): The computed mean range to the object, along with a timestamp.
    """

    def __init__(self):
        """
        Initializes the ObjectRange node, sets up subscribers and publishers.
        """
        super().__init__('object_range')

        self.angle_data = None
        self.lidar_data = None

        # Subscriber to the object's angle(s)
        self.angle_subscriber = self.create_subscription(
            Float32MultiArray, '/object_angle', self.angle_callback, 10
        )

        # Subscriber to lidar scan data
        self.lidar_subscriber = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile
        )

        # Publisher for object range data
        self.range_publisher = self.create_publisher(Float32MultiArray, 'object_range', 10)

    def angle_callback(self, msg):
        """
        Callback function for handling incoming object angle data.

        Args:
            msg (Float32MultiArray): Object angle information. Expected structure:
                                      [timestamp, flag, angle1, angle2, ...]
        """
        self.angle_data = msg.data
        # Process only if the object is being tracked (flag == 1.0)
        if self.angle_data[1] == 1.0:
            self.compute_range()

    def lidar_callback(self, msg):
        """
        Callback function for handling incoming lidar scan data.

        Args:
            msg (LaserScan): Lidar scan data containing distances and angles.
        """
        self.lidar_data = msg
        self.compute_range()

    def compute_range(self):
        """
        Computes the range to the detected object based on lidar data and object angles.
        For each angle in the message, the corresponding lidar distance is extracted.
        Outlier distances are filtered out using an IQR-based approach, and the mean of the remaining
        distances is computed as the final reported range.
        """
        if self.lidar_data is None or self.angle_data is None:
            # Wait until both lidar and angle data are available.
            return

        # Extract the object's angles. Assumes that angles start at index 2.
        angles = self.angle_data[2:]
        lidar_msg = self.lidar_data
        valid_distances = []
        range_min = lidar_msg.range_min
        range_max = lidar_msg.range_max

        for angle in angles:
            # Adjust the angle to match the lidar's coordinate frame.
            # (Assumes lidar scan 0 is at the front; adjust if needed.)
            adjusted_angle = -angle + np.pi

            # Compute the index into the lidar scan array.
            index = int((adjusted_angle - lidar_msg.angle_min) / lidar_msg.angle_increment)

            # Check if the index is within the valid range.
            if index < 0 or index >= len(lidar_msg.ranges):
                self.get_logger().warning(f"Angle {angle:.3f} (adjusted: {adjusted_angle:.3f}) yields invalid index {index}.")
                continue

            dist = lidar_msg.ranges[index]

            # Validate the distance measurement.
            if dist < range_min or dist > range_max or np.isnan(dist) or np.isinf(dist):
                self.get_logger().debug(f"Distance {dist} at index {index} is out of range.")
                continue

            valid_distances.append(dist)

        # If we found valid distances, filter out outliers.
        if valid_distances:
            distances_arr = np.array(valid_distances)

            # If enough samples exist, use the IQR method to filter out outliers.
            if len(distances_arr) >= 3:
                Q1 = np.percentile(distances_arr, 25)
                Q3 = np.percentile(distances_arr, 75)
                IQR = Q3 - Q1
                lower_bound = Q1 - 1.5 * IQR
                upper_bound = Q3 + 1.5 * IQR
                filtered = distances_arr[(distances_arr >= lower_bound) & (distances_arr <= upper_bound)]
                if filtered.size > 0:
                    mean_distance = float(np.mean(filtered))
                else:
                    # If filtering removes all samples, fall back to the mean of all distances.
                    mean_distance = float(np.mean(distances_arr))
            else:
                # If only one or two samples are available, compute the mean directly.
                mean_distance = float(np.mean(distances_arr))
        else:
            # No valid distances were found.
            mean_distance = -1.0

        # Build and publish the range message.
        range_msg = Float32MultiArray()
        # Use the lidar header's timestamp if available.
        timestamp = float(lidar_msg.header.stamp.sec + lidar_msg.header.stamp.nanosec * 1e-9)
        # The message structure is: [timestamp, flag, mean_distance]
        flag = 1.0 if mean_distance != -1.0 else 0.0
        range_msg.data = [timestamp, flag, mean_distance]

        self.get_logger().info(f"Computed Mean Range: {mean_distance:.3f} (from distances: {valid_distances})")
        self.range_publisher.publish(range_msg)

def main(args=None):
    """
    Initializes the ROS2 node and spins to keep it active for processing callbacks.
    """
    rclpy.init(args=args)
    node = ObjectRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
