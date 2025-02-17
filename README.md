# ros-turtlebot3

**ros-turtlebot3** is a suite of ROS2 packages designed for autonomous object detection, tracking, and following. This project demonstrates robust integration of computer vision, lidar sensing, and control algorithms to enable a robot to detect, track, and follow objects in dynamic environments. The package is developed with modularity in mind, facilitating future extensions for more complex tasks such as advanced path planning, obstacle avoidance, and multi-object tracking.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Packages](#packages)
- [Installation](#installation)
- [Dependencies](#dependencies)
- [Future Work](#future-work)
- [Contributors](#contributors)
- [License](#license)

---

## Overview

The **Object Follower** project is focused on enabling robots to autonomously follow a moving object by combining sensory inputs and applying control algorithms. The system utilizes:

- **Computer Vision** for object detection and angle estimation.
- **Lidar data** for accurate distance measurements.
- **PID controllers** to generate smooth and stable motion commands for both angular and linear movement.

The project currently includes two key packages:
- **object_follower**: The latest and most advanced package which integrates multiple sensor modalities (camera and lidar) to provide robust tracking and following.
- **object_tracker**: A simpler package that demonstrates basic object tracking by rotating the robot to align with a detected object.

---

## Features

- **Multi-Sensor Fusion**: Combines camera and lidar data for accurate object localization.
- **Real-Time Object Tracking**: Detects and tracks objects using HSV masking and contour detection.
- **PID Control**: Implements PID controllers to generate smooth motion commands.
- **Modular Design**: The project is structured to support future enhancements and additional packages for more complex tasks.
- **ROS2 Integration**: Built using ROS2, ensuring scalability and compatibility with modern robotic systems.

---

## Packages

### object_follower

The **object_follower**
- Uses camera images to detect objects via HSV masking.
- Computes the object's angular position relative to the camera's center.
- Converts camera angle measurements to match lidar coordinate conventions.
- Processes lidar data to determine the object's range.
- Uses PID control to generate velocity commands that ensure smooth following behavior.

### object_tracker

The **object_tracker** package is a simpler implementation focused on:
- Rotating the robot to align with a tracked object.
- Serving as a foundation for basic tracking tasks and as a learning tool for understanding sensor integration and control in ROS2.

---

## Installation

### Prerequisites

- **ROS2 Foxy** (or later)
- **Python 3.8+**
- **OpenCV**
- **cv_bridge**
- **numpy**

### Steps

1. **Clone the repository:**

   ```bash
   git clone https://github.com/yourusername/object_follower.git
   cd object_follower
   ```

2. **Build the ROS2 workspace:**

   ```bash
   colcon build
   ```

3. **Source the setup file:**

   ```bash
   source install/setup.bash
   ```

---

## Dependencies

- **ROS2 (rclpy, sensor_msgs, std_msgs, geometry_msgs)**
- **OpenCV** for image processing and computer vision.
- **cv_bridge** for converting between ROS image messages and OpenCV images.
- **numpy** for numerical operations and angle conversions.

---

## Future Work

The **Object Follower** project is designed with future enhancements in mind. Upcoming packages and features include:
- **Advanced Path Planning**: Algorithms for obstacle avoidance and dynamic re-routing.
- **Multi-Object Tracking**: Capability to detect and track multiple objects simultaneously.
- **Sensor Fusion Enhancements**: Integration of additional sensor modalities (e.g., depth cameras, IMUs) for improved robustness.
- **Machine Learning Integration**: Use of neural networks for improved object recognition and predictive tracking.

---

## License

This project is licensed under the [Apache 2.0 License](LICENSE).

---

*Object Follower* is a comprehensive project that demonstrates cutting-edge techniques in robotics and autonomous systems. Its modular design and future-ready architecture make it an excellent addition to any robotics portfolio or resume.

For more information, please visit [GitHub Repository](https://github.com/yourusername/object_follower). 

Feel free to reach out for collaboration or if you have any questions regarding the project!

---
