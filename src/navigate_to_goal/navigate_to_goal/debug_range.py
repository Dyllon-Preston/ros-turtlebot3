#!/usr/bin/env python3
"""
Debug Range Node (debug_range.py):
Subscribes to obstacle vector data and displays an image that shows the different distance vectors 
relative to the robot's local coordinate frame.

The node expects data from the /obstacle_vectors topic as a flattened Float32MultiArray in the format:
[x1, y1, z1, x2, y2, z2, ..., xN, yN, zN]

The image shows:
- The robot at the center (green circle)
- Each detected obstacle (red circle)
- A line from the robot to each obstacle (cyan line)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from std_msgs.msg import Float32MultiArray

class DebugRange(Node):
    def __init__(self):
        super().__init__('debug_range')
        # Subscribe to the obstacle vector topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/obstacle_vectors',
            self.obstacle_callback,
            10
        )
        
        # Define image parameters
        self.img_size = 600          # Create a 600x600 pixel image
        self.img_center = self.img_size // 2
        self.scale = 100             # Scale: 100 pixels per meter
        
        # Create an OpenCV window to display the debug image
        cv2.namedWindow("Debug Range", cv2.WINDOW_NORMAL)

    def obstacle_callback(self, msg: Float32MultiArray):
        # Convert the flattened array to an (N x 3) numpy array
        data = msg.data
        if len(data) % 3 != 0:
            self.get_logger().error("Received obstacle vector data with invalid length.")
            return
        obstacles = np.array(data).reshape(-1, 3)
        
        # Create a blank image (black background)
        img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        
        # Draw the robot's position at the center (green circle)
        robot_color = (0, 255, 0)
        cv2.circle(img, (self.img_center, self.img_center), 5, robot_color, -1)
        
        # Plot each obstacle vector relative to the robot
        for vec in obstacles:
            x, y, _ = vec  # z is ignored since we're working in 2D
            # Convert coordinates (in meters) to pixel positions.
            # Note: In the image, x increases to the right and y increases downward.
            pixel_x = int(self.img_center + x * self.scale)
            pixel_y = int(self.img_center - y * self.scale)
            
            # Draw the obstacle as a red circle
            obstacle_color = (0, 0, 255)
            cv2.circle(img, (pixel_x, pixel_y), 5, obstacle_color, -1)
            
            # Draw a line from the robot (center) to the obstacle (cyan line)
            cv2.line(img, (self.img_center, self.img_center), (pixel_x, pixel_y), (255, 255, 0), 2)
        
        # Display the debug image
        cv2.imshow("Debug Range", img)
        cv2.waitKey(1)  # Update the window

def main(args=None):
    rclpy.init(args=args)
    node = DebugRange()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
