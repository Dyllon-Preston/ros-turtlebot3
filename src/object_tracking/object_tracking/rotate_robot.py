#!/usr/bin/env python3

"""
Rotate robot so that the object center is within the center of the camera (within some tolerance).

Group Name: Levi
Group Members:
- Dyllon Preston
- Richi Dubey
"""

import rclpy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

class RotateRobot(Node):
    def __init__(self):
        super().__init__('track_object')

        # Parameters
        self.image_width = None # Variable to store image width
        self.center_threshold = 0.05  # 5% deviation allowed

        # Subscribers & Publishers
        # Subscribe to the tracked object's center
        self.subscription = self.create_subscription(
            Int32MultiArray, '/object_center', self.object_center_callback, 10)
        # Subscribe to the camera image
        self._img_subscriber = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self._image_callback, 10)
        # Publish cmd_vel rotations
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        
    def _image_callback(self, msg):
        # Convert the compressed image data to an OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image is not None:
            # Set image width dynamically
            self.image_width = image.shape[1]  # Get the width of the image
        else:
            self.get_logger().warn("Failed to decode the image.")

    def object_center_callback(self, msg):
        if len(msg.data) < 2:
            return  # Invalid message
        if self.image_width is None:
            self.get_logger().warn("Image width not received yet.")
            return

        object_x = msg.data[0]
        image_center_x = self.image_width // 2
        deviation = abs(object_x - image_center_x) / self.image_width

        twist_msg = Twist()

        if deviation > self.center_threshold:
            if object_x < image_center_x:
                twist_msg.angular.z = 1.0  # Turn left
                self.get_logger().info("Turning left")
            else:
                twist_msg.angular.z = -1.0  # Turn right
                self.get_logger().info("Turning right")
        else:
            twist_msg.angular.z = 0.0  # No movement
            self.get_logger().info("Holding position")

        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RotateRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
