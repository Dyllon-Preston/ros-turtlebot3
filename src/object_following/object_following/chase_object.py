#!/usr/bin/env python3

"""
Chase Object Node: Rotates the robot so that the object center is aligned with the center of the camera 
within a certain tolerance. The node uses PID control for both angle and range adjustment.

Group Name: Levi
Group Members:
- Dyllon Preston
- Richi Dubey

This ROS2 node subscribes to the object's angle and range and publishes velocity commands 
to align the object with the center of the robot's camera view.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class PID:
    """
    A simple PID controller for controlling the robot's movements.

    Parameters:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
        delay (float): Delay time to prevent unstable responses (default: 0.1 seconds).
    """

    def __init__(self, Kp, Ki, Kd, delay=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.t_prev = None
        self.output_prev = 0.0
        self.delay = delay
    
    def reset(self):
        """Resets the PID controller's integral and previous error values."""
        self.integral = 0
        self.prev_error = 0.0
    
    def __call__(self, error, t):
        """
        Computes the PID output based on the current error and time.

        Args:
            error (float): The current error to be corrected.
            t (float): The current time (timestamp of the message).

        Returns:
            float: The computed PID output (effort to apply).
        """
        if self.t_prev is None:
            self.t_prev = t
            return self.Kp * error
        
        dt = t - self.t_prev
        if dt < 0.0:
            return 0.0
        elif dt < self.delay:
            return self.output_prev
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        
        self.prev_error = error
        self.t_prev = t
        self.output_prev = output
        
        return output


class ChaseObject(Node):
    """
    ROS2 Node that uses PID control to adjust the robot's movement to center an object in the camera view.

    Subscribes to:
    - /object_angle (Float32MultiArray): The current angle to the object from the robot's center.
    - /object_range (Float32MultiArray): The current range to the object from the robot.

    Publishes:
    - /cmd_vel (Twist): The velocity commands to the robot to adjust its position and orientation.
    """

    def __init__(self):
        """
        Initializes the ChaseObject node with PID controllers for both angle and range control.
        """
        super().__init__('track_object')

        # Create PID controllers for angle and range adjustments
        self.angle_PID = PID(Kp=0.02, Ki=0.02, Kd=0.02)
        self.range_PID = PID(Kp=0.01, Ki=0.01, Kd=0.01)
        
        self.angle_data = None
        self.range_data = None
        self.target_distance = 0.2  # Target distance to the object (meters)

        # Subscribers & Publishers
        # Subscribe to object angle data
        self.angle_subscriber = self.create_subscription(
            Float32MultiArray, '/object_angle', self.object_angle_callback, 10
        )
        # Subscribe to object range data
        self.range_subscriber = self.create_subscription(
            Float32MultiArray, '/object_range', self.object_range_callback, 10
        )
        # Publish velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def object_angle_callback(self, msg):
        """Callback for object angle data."""
        self.angle_data = msg.data
        self.chase()

    def object_range_callback(self, msg):
        """Callback for object range data."""
        self.range_data = msg.data
        self.chase()

    def chase(self):
        """
        Controls the robot's movement to chase the object by using PID controllers 
        to adjust both the robot's angle and range relative to the object.
        """
        if self.angle_data is None or self.range_data is None:
            return
        
        # If no object or invalid data, stop the robot
        if self.angle_data[1] == 0.0 or self.range_data[1] == 0.0:
            twist = Twist()
            twist.angular.z = 0
            twist.linear.x = 0
            self.publisher.publish(twist)
            return

        # Compute the error in angle and range
        angle_error = -self.angle_data[2]
        range_error = self.range_data[2] - self.target_distance
        # Use the timestamp from the lidar data for both angle and range time
        t_angle = self.range_data[0]
        t_range = self.range_data[0]

        # Log the errors for debugging
        self.get_logger().info(f"Angle Error: {angle_error} | Range Error: {range_error} | Time: {t_range}")

        # If valid angle data, compute control efforts
        if self.angle_data[1] == 1.0:
            angle_effort = self.angle_PID(angle_error, t_angle)
            range_effort = self.range_PID(range_error, t_range)

            # Create and publish the Twist message
            twist = Twist()
            twist.angular.z = angle_effort
            twist.linear.x = range_effort
            self.publisher.publish(twist)
        else:
            # Reset PID controllers if no valid data
            self.angle_PID.reset()
            self.range_PID.reset()

            # Stop the robot if no valid object is tracked
            twist = Twist()
            twist.angular.z = 0
            twist.linear.x = 0
            self.publisher.publish(twist)

        # Log the twist command for debugging
        self.get_logger().info(f"Twist Command: {twist}")


def main(args=None):
    """
    Initializes the ROS2 node and spins the event loop for callbacks.
    """
    rclpy.init(args=args)
    node = ChaseObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
