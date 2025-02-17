#!/usr/bin/env python3

"""
Object Selection Node: Allows a user to select an HSV value from an image by clicking on a pixel, 
and then publishes the selected HSV value for object tracking.

Group Name: Levi
Group Members:
- Dyllon Preston
- Richi Dubey

This ROS2 node subscribes to the compressed camera feed and tracked image feed, 
allows the user to select a pixel on the image to get its HSV value, and publishes the selected HSV value 
to the '/selected_hsv' topic for object tracking.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
import cv2
from cv_bridge import CvBridge

class SelectObject(Node):
    """
    ROS2 Node for selecting an object in an image based on its HSV value.

    Subscribes to:
    - /image_raw/compressed (CompressedImage): Compressed image from the camera.
    - /tracked_image/compressed (CompressedImage): Processed image with tracked object annotations.

    Publishes:
    - /selected_hsv (Int32MultiArray): The selected HSV value for object detection.
    """

    def __init__(self):
        """
        Initializes the SelectObject node, sets up subscriptions and publishers, and prepares for user interaction.
        """
        super().__init__('select_object')

        # Subscribers to the camera and tracked image topics
        self._img_subscriber = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self._image_callback, 10
        )
        self._tracked_img_subscriber = self.create_subscription(
            CompressedImage, '/tracked_image/compressed', self._tracked_image_callback, 10
        )

        # Publisher for the selected HSV value
        self.hsv_publisher = self.create_publisher(Int32MultiArray, '/selected_hsv', 10)

        # Initialize CVBridge for converting ROS messages to OpenCV images
        self.bridge = CvBridge()

        # Initialize variables to store frames
        self.selected_hsv = None
        self.current_frame = None
        self.tracked_frame = None

        # Create OpenCV window and set up mouse click callback
        cv2.namedWindow("Select Object")
        cv2.setMouseCallback("Select Object", self.get_hsv_on_click)
    
    def get_hsv_on_click(self, event, x, y, flags, param):
        """
        Callback function that captures the HSV value of a pixel on mouse click.

        Args:
            event (int): The type of mouse event (e.g., left-click).
            x (int): The x-coordinate of the mouse click.
            y (int): The y-coordinate of the mouse click.
            flags (int): Any flags that are passed to the callback.
            param (object): Additional parameters passed to the callback.
        """
        if event == cv2.EVENT_LBUTTONDOWN and self.current_frame is not None:
            # Convert the selected pixel to HSV color space
            hsv_frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
            self.selected_hsv = hsv_frame[y, x]
            
            # Publish the selected HSV value
            msg = Int32MultiArray(data=self.selected_hsv.tolist())
            self.hsv_publisher.publish(msg)
            self.get_logger().info(f"Selected HSV: {self.selected_hsv}")
    
    def _image_callback(self, msg):
        """
        Callback function for receiving the raw camera image, processing it, and updating the display.

        Args:
            msg (CompressedImage): The compressed image message containing the camera feed.
        """
        # Convert the compressed image message to an OpenCV image
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.current_frame = frame
        self._update_display()

    def _tracked_image_callback(self, msg):
        """
        Callback function for receiving the processed (tracked) image, updating the display with the tracked image.

        Args:
            msg (CompressedImage): The compressed image message containing the tracked object annotations.
        """
        self.tracked_frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._update_display()

    def _update_display(self):
        """
        Updates the OpenCV display window based on the available frames.
        If an HSV value has been selected, the tracked image is displayed; 
        otherwise, the raw camera frame is displayed.
        """
        if self.selected_hsv is not None and self.tracked_frame is not None:
            # Display the tracked frame if HSV is selected and tracked frame is available
            cv2.imshow("Select Object", self.tracked_frame)
        elif self.current_frame is not None:
            # Display the raw frame if no tracked frame is available
            cv2.imshow("Select Object", self.current_frame)

        cv2.waitKey(1)

def main(args=None):
    """
    Initializes the ROS2 node, starts the spinning loop, and handles callbacks.
    """
    rclpy.init(args=args)
    node = SelectObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
