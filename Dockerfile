# Dockerfile for ROS with Gazebo, Cartographer, Navigation2, and TurtleBot3
# Generated by Dyllon Preston on 1/11/2025 for AE 7785

# The base image specifies the foundation of the Docker container.
# "ros:humble-ros-core-jammy" includes the core ROS2 Humble packages
# on an Ubuntu Jammy (22.04) base. This ensures compatibility between
# ROS and the operating system.
FROM ros:humble-ros-core-jammy

# Install basic development tools and ROS dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Initialize and update rosdep (ROS dependency manager)
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# Configure colcon for building ROS2 packages
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# Install core ROS2 base packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo simulator and plugins
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-* \
    && rm -rf /var/lib/apt/lists/*

# Install Cartographer for SLAM
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    && rm -rf /var/lib/apt/lists/*

# Install Navigation2 for robot path planning
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Install TurtleBot3 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*

# Set up TurtleBot3 workspace and required packages
RUN mkdir -p ~/turtlebot3_ws/src && \
    cd ~/turtlebot3_ws/src && \
    git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
    apt-get update && apt-get install -y --no-install-recommends python3-colcon-common-extensions && \
    cd ~/turtlebot3_ws && \
    bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install" && \
    echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc && \
    bash -c "source ~/.bashrc"

# Create and build the ROS2 workspace (ros2_humble)
RUN mkdir -p ~/ros2_humble/src && \
    cd ~/ros2_humble && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src && \
    apt-get update && apt-get upgrade -y && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" && \
    colcon build --symlink-install

# Configure ROS environment for remote operation and Gazebo integration
RUN echo 'source ~/ros2_humble/install/setup.bash' >> ~/.bashrc && \
    echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'export ROS_DOMAIN_ID=54' >> ~/.bashrc && \
    echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc && \
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc && \
    /bin/bash -c "source ~/.bashrc"

# Install some useful packages
# nano for editing text files
# net-tools for ifconfig
# ping for checking connectivity
RUN sudo apt update && sudo apt install -y \
    nano \
    net-tools \
    inetutils-ping

# Additional Notes:
# - This Dockerfile is designed for AE 7785 coursework and includes setup for TurtleBot3 simulation.
# - An X server is required for Gazebo GUI.
# - To build the Docker image, run "docker build -t ros_humble_ae7785 ."
# - To run the Docker container with GUI support, run "docker run -e DISPLAY=host.docker.internal:0 -it ros_humble_ae7785"