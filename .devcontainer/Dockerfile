FROM ros:humble-ros-base

ARG ROS_DISTRO=humble

ARG USERNAME=leonv
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container
RUN if id -u $USER_UID >/dev/null 2>&1; then userdel -f `id -un $USER_UID`; fi

# Create non-root user with sudo, i2c, and dialout support
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && groupadd -f i2c \
    && usermod -aG i2c,dialout $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    i2c-tools \
    libi2c-dev \
    pybind11-dev \
    python3-dev \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-rosbridge-suite \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-rqt-image-view \
    v4l-utils \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install modern CMake for PiPCA9685 compatibility
RUN pip3 install --no-cache-dir cmake

# Install libraries for INA3221
RUN pip3 install --no-cache-dir adafruit-circuitpython-ina3221 adafruit-blinka

# Install libraries for MPU6500
RUN pip3 install --no-cache-dir smbus2

# Install Ultralytics YOLO (pinned numpy<2 and opencv-python<5 for ROS 2 Humble cv_bridge compatibility)
RUN pip3 install --no-cache-dir "setuptools<80" "packaging>=22.0" "numpy<2" "opencv-python<5" ultralytics

# Build and install PiPCA9685 library
WORKDIR /opt
RUN git clone https://github.com/barulicm/PiPCA9685.git \
    && cd PiPCA9685 \
    && mkdir build && cd build \
    && cmake .. \
    && make \
    && make install

# Auto-source ROS 2 setup for bash shells
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo "if [ -f /home/ws/install/setup.bash ]; then source /home/ws/install/setup.bash; fi" >> /etc/bash.bashrc

WORKDIR /home/ws
USER $USERNAME
CMD ["/bin/bash"]
