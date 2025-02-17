#!/usr/bin/env python3

"""
Uses HSV masking to find the center of an object in an image and publish that center.

Group Name: Levi
Group Members:
- Dyllon Preston
- Richi Dubey
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge


class FindObject(Node):
    def __init__(self):
        super().__init__('find_object')
        # Subscribe to the camera image
        self._img_subscriber = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self._image_callback, 10)
        # Subscribe to the tracked HSV value
        self.hsv_subscriber = self.create_subscription(
            Int32MultiArray, '/selected_hsv', self._hsv_callback, 10)
        # Publish the center of the tracked object
        self.center_publisher = self.create_publisher(Int32MultiArray, '/object_center', 10)
        
        self.bridge = CvBridge()
        self.selected_hsv = None # Initialize stored HSV value
    
    def _hsv_callback(self, msg):
        self.selected_hsv = np.array(msg.data, dtype=np.uint8)
        self.get_logger().info(f"Received HSV: {self.selected_hsv}")
    
    def _image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.selected_hsv is not None:
            # Convert frame to HSV
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define HSV bounds
            lower_bound = np.clip(self.selected_hsv - [10, 50, 50], 0, 255)
            upper_bound = np.clip(self.selected_hsv + [10, 50, 50], 0, 255)

            # Create mask
            mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Find the largest contour by area
                largest_contour = max(contours, key=cv2.contourArea)

                if cv2.contourArea(largest_contour) > 500:  # Threshold to filter noise
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    center = [x + w // 2, y + h // 2]

                    # Publish the center of the largest object
                    msg = Int32MultiArray(data=center)
                    self.center_publisher.publish(msg)

                    # Draw bounding box and center point
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)

                    self.get_logger().info(f"Object Center: {center}")

        cv2.imshow("Find Object", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FindObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()