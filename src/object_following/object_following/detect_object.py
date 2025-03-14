#!/usr/bin/env python3

"""
Object Detection Node: Uses HSV masking to find the center of a tracked object in a camera image and
publishes the local angles to the object along its width and the tracked image.

Group Name: Levi
Group Members:
- Dyllon Preston
- Richi Dubey

This ROS2 node subscribes to the camera image (compressed), applies an HSV mask to identify the object, computes
sampled angles across the object's width relative to the camera's field of view, and publishes the angles along with the
processed image (containing the bounding box and sample points of the detected object).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray, Float64MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge

class DetectObject(Node):
    """
    ROS2 Node for detecting an object in an image based on a tracked HSV value.

    Subscribes to:
    - /image_raw/compressed (CompressedImage): Compressed image from the camera.
    - /selected_hsv (Int32MultiArray): HSV values used to identify the object.

    Publishes:
    - /object_angle (Float64MultiArray): The computed angles to the object relative to the camera's field of view.
    - /tracked_image/compressed (CompressedImage): The image with the tracked object, annotated with a bounding box and sample points.
    """

    def __init__(self):
        """
        Initializes the DetectObject node, sets up subscribers and publishers, and prepares the camera's field of view.
        """
        super().__init__('detect_object')

        # Subscribers to image and HSV data
        self._img_subscriber = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self._image_callback, 10
        )
        self.hsv_subscriber = self.create_subscription(
            Int32MultiArray, '/selected_hsv', self._hsv_callback, 10
        )

        # Publishers for the object angles and tracked image
        self.angle_publisher = self.create_publisher(Float64MultiArray, '/object_angle', 10)
        self.tracked_image_publisher = self.create_publisher(CompressedImage, '/tracked_image/compressed', 10)

        # Initialize CVBridge for converting ROS messages to OpenCV images
        self.bridge = CvBridge()
        self.selected_hsv = None  # Initialize stored HSV value

        # Camera field of view (in radians)
        self.fov = 62.2 * np.pi / 180  # Camera field of view in radians

        # Number of samples along the detected object’s width
        self.sample_count = 11

    def _hsv_callback(self, msg):
        """
        Callback function for receiving the selected HSV value for object tracking.

        Args:
            msg (Int32MultiArray): The HSV value for object detection.
        """
        self.selected_hsv = np.array(msg.data, dtype=np.uint8)
        self.get_logger().info(f"Received HSV: {self.selected_hsv}")

    def _image_callback(self, msg):
        """
        Callback function for processing incoming image data, detecting objects based on HSV, 
        sampling angles along the object's width, and publishing the angles and annotated image.

        Args:
            msg (CompressedImage): The compressed image message containing the camera feed.
        """
        # Convert the compressed image message to an OpenCV image
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.selected_hsv is not None:
            # Convert the frame from BGR to HSV color space
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define HSV bounds for masking
            lower_bound = np.clip(self.selected_hsv - [10, 50, 50], 0, 255)
            upper_bound = np.clip(self.selected_hsv + [10, 50, 50], 0, 255)

            # Create a mask for the object based on the HSV range
            mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Find the largest contour by area
                largest_contour = max(contours, key=cv2.contourArea)

                if cv2.contourArea(largest_contour) > 500:  # Threshold to filter noise
                    # Get bounding box of the largest contour
                    x, y, w, h = cv2.boundingRect(largest_contour)

                    # Initialize list to store sampled angles
                    sampled_angles = []

                    # Sample angles along the object's width
                    for i in range(self.sample_count):
                        # Compute the x-coordinate for this sample (evenly spaced along the width)
                        sample_x = x + (w * i) / (self.sample_count - 1)
                        # Compute the angle relative to the center of the image
                        angle_sample = (sample_x - frame.shape[1] // 2) * self.fov / frame.shape[1]
                        # Normalize the angle between -pi and pi
                        angle_sample = np.arctan2(np.sin(angle_sample), np.cos(angle_sample))
                        sampled_angles.append(angle_sample)

                        # Annotate the sample point on the frame (using the vertical center of the bounding box)
                        cv2.circle(frame, (int(sample_x), int(y + h / 2)), 3, (255, 0, 0), -1)

                    # Publish angle information.
                    # The message data consists of a timestamp, a flag (1.0 indicating detection), followed by the sampled angles.
                    angle_msg = Float64MultiArray()
                    timestamp = self.get_clock().now().nanoseconds * 1e-9
                    angle_msg.data = [timestamp, 1.0] + sampled_angles
                    self.angle_publisher.publish(angle_msg)

                    # Annotate the frame with a bounding box
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            else:
                # If no object is detected, publish zeros.
                angle_msg = Float64MultiArray()
                timestamp = self.get_clock().now().nanoseconds * 1e-9
                angle_msg.data = [timestamp, 0.0] + [0.0] * self.sample_count
                self.angle_publisher.publish(angle_msg)
                self.get_logger().info("No object found")

        # Convert the annotated image back to a compressed image message and publish it
        tracked_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.tracked_image_publisher.publish(tracked_image_msg)

def main(args=None):
    """
    Initializes the ROS2 node, spins to keep it active, and handles callbacks.
    """
    rclpy.init(args=args)
    node = DetectObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
