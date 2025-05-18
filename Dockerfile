FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Optional: install Python packages you may use
RUN pip3 install --no-cache-dir opencv-python numpy
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi" >> ~/.bashrc


# Set working directory
WORKDIR /root/ros2_ws

# Source ROS every shell session
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
