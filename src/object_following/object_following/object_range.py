#!/usr/bin/env python3

"""
ObjectRange Node: Computes the range to a detected object using lidar data and the object's angle.

Group Name: Levi
Group Members:
- Dyllon Preston
- Richi Dubey

This ROS2 node subscribes to the object's angle and lidar scan data, computes the range of the detected object,
and publishes the range along with a timestamp. The lidar data is indexed based on the angle of the object.

Dependencies:
- rclpy: ROS2 Python client library
- sensor_msgs.msg: For receiving LaserScan data
- std_msgs.msg: For publishing Float32MultiArray (used to send range data)
- numpy: For mathematical operations (angle conversion)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class ObjectRange(Node):
    """
    ROS2 Node for computing the range to an object based on lidar data and object's angle.

    Subscribes to:
    - /object_angle (Float32MultiArray): The angle at which the object is detected
    - /scan (LaserScan): Lidar scan data for distance measurements

    Publishes:
    - object_range (Float32MultiArray): The range to the object, along with a timestamp
    """

    def __init__(self):
        """
        Initializes the ObjectRange node, sets up subscribers and publishers.
        """
        super().__init__('object_range')

        self.angle_data = None
        self.lidar_data = None

        # Subscriber to the object's angle
        self.angle_subscriber = self.create_subscription(
            Float32MultiArray, '/object_angle', self.angle_callback, 10
        )

        # Subscriber to lidar scan data
        self.lidar_subscriber = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )

        # Publisher for object range data
        self.range_publisher = self.create_publisher(Float32MultiArray, 'object_range', 10)

    def angle_callback(self, msg):
        """
        Callback function for handling incoming object angle data.

        Args:
            msg (Float32MultiArray): Object angle information
        """
        self.angle_data = msg.data
        if self.angle_data[1] == 1:  # Validates if the object is being tracked
            self.compute_range()

    def lidar_callback(self, msg):
        """
        Callback function for handling incoming lidar scan data.

        Args:
            msg (LaserScan): Lidar scan data containing distances and angles
        """
        self.lidar_data = msg
        self.compute_range()

    def compute_range(self):
        """
        Computes the range to the detected object based on the lidar data and object angle.
        The angle is adjusted to match the lidar data's coordinate system, and the range is
        calculated by indexing the lidar scan at the appropriate angle.
        """
        if self.lidar_data is None or self.angle_data is None:
            # Wait for both angle and lidar data to be available
            return
        
        angle = self.angle_data[2]
        lidar_data = self.lidar_data

        # Adjust the angle to match the lidar's reference frame (assuming lidar data starts from the front)
        angle = -angle + np.pi

        # Compute the lidar scan index corresponding to the given angle
        index = int((angle - lidar_data.angle_min) / lidar_data.angle_increment)

        # Ensure the index is within the valid range
        if index < 0 or index >= len(lidar_data.ranges):
            msg = Float32MultiArray()
            timestamp = float(lidar_data.header.stamp.sec + lidar_data.header.stamp.nanosec * 1e-9)
            msg.data = [timestamp, 0.0, -1.0]  # Invalid range, -1 indicates no valid detection
        else:
            msg = Float32MultiArray()

            dist = lidar_data.ranges[index]

            range_min = lidar_data.range_min
            range_max = lidar_data.range_max

            if dist < range_min or dist > range_max:
                timestamp = float(lidar_data.header.stamp.sec + lidar_data.header.stamp.nanosec * 1e-9)
                msg.data = [timestamp, 0.0, -1.0]  # Valid range
            else:
                timestamp = float(lidar_data.header.stamp.sec + lidar_data.header.stamp.nanosec * 1e-9)
                msg.data = [timestamp, 1.0, lidar_data.ranges[index]]  # Valid range

        self.get_logger().info(f"Object Range: {msg.data[2]}")

        # Publish the computed range along with the timestamp
        self.range_publisher.publish(msg)

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
