#!/usr/bin/env python3

"""
Node for selecting the HSV value from the pixel of an image, and displaying the results of find_object.py.

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
from cv_bridge import CvBridge

class SelectObject(Node):
    def __init__(self):
        super().__init__('select_object')
        # Subscribe to the camera image
        self._img_subscriber = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self._image_callback, 10)
         # Subscribe to the tracked image
        self._tracked_img_subscriber = self.create_subscription(
            CompressedImage, '/tracked_image/compressed', self._tracked_image_callback, 10)
        # Publish tracked HSV value
        self.hsv_publisher = self.create_publisher(Int32MultiArray, '/selected_hsv', 10)
        self.bridge = CvBridge()
        self.selected_hsv = None
        self.current_frame = None
        self.tracked_frame = None
        cv2.namedWindow("Select Object")
        cv2.setMouseCallback("Select Object", self.get_hsv_on_click)
    
    def get_hsv_on_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.current_frame is not None:
            hsv_frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
            self.selected_hsv = hsv_frame[y, x]
            msg = Int32MultiArray(data=self.selected_hsv.tolist())
            self.hsv_publisher.publish(msg)
            self.get_logger().info(f"Selected HSV: {self.selected_hsv}")
    
    def _image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.current_frame = frame
        self._update_display()

    def _tracked_image_callback(self, msg):
        """Handle incoming processed (tracked) images."""
        self.tracked_frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._update_display()

    def _update_display(self):
        """Update the displayed image based on whether HSV is selected."""
        if self.selected_hsv is not None and self.tracked_frame is not None:
            cv2.imshow("Select Object", self.tracked_frame)
        elif self.current_frame is not None:
            cv2.imshow("Select Object", self.current_frame)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SelectObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()