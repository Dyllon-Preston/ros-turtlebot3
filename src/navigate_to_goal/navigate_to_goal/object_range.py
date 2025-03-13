import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT
)

class ObjectRange(Node):

    """
    ROS2 Node for detecting objects using LiDAR data based on gaps in scan ranges.
    
    Subscribes to:
    - /scan (LaserScan): LiDAR scan data containing distance measurements.
    
    Publishes:
    - /object_positions (Float64MultiArray): List of detected object positions in [x, y, z] format.
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
            Float64MultiArray, '/obstacle_vectors', 10
        )

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

        # Filter out invalid readings
        valid_mask = (ranges > range_min) & (ranges < range_max)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        if valid_ranges.size == 0:
            self.get_logger().warn("No valid obstacles detected.")
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
            z = 0  # No height information from 2D LiDAR
            obstacle_vectors.extend([x, y, z])  # Flatten into a list

        # Publish the ordered list of obstacle vectors as a Float64MultiArray
        msg = Float64MultiArray()
        msg.data = obstacle_vectors
        self.obstacle_publisher.publish(msg)

        self.get_logger().info(f'Published {len(obstacle_vectors) // 3} sorted obstacle vectors.')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
