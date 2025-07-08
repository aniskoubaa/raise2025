# RAISE 2025 - Development Environment
# Based on ROS2 Humble with AI/CV tools

FROM ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PYTHONUNBUFFERED=1

# Update system and install basic tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    curl \
    git \
    nano \
    vim \
    wget \
    software-properties-common \
    build-essential \
    cmake \
    pkg-config \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran \
    openexr \
    libatlas-base-dev \
    libtbb2 \
    libtbb-dev \
    libdc1394-22-dev \
    libopenexr-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Install additional ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-rqt* \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-msgs \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /workspace
WORKDIR /workspace

# Set up ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export PYTHONPATH=/workspace:$PYTHONPATH" >> ~/.bashrc

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create a non-root user
RUN useradd -m -s /bin/bash -G sudo raise2025
RUN echo "raise2025:raise2025" | chpasswd
RUN echo "raise2025 ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to non-root user
USER raise2025
WORKDIR /home/raise2025

# Set up ROS2 environment for user
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export PYTHONPATH=/workspace:$PYTHONPATH" >> ~/.bashrc

# Copy startup script
COPY --chown=raise2025:raise2025 start.sh /home/raise2025/start.sh
RUN chmod +x /home/raise2025/start.sh

# Set default command
CMD ["/home/raise2025/start.sh"] 