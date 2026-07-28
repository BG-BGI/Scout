FROM ros:humble-ros-core

# System Dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    libboost-dev \
    python3-colcon-common-extensions \
    python3-spidev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ROS Packages
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    ros-humble-rclcpp \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# roboclaw_driver from source
RUN mkdir -p /opt/src_ws/src \
    && git clone --depth 1 https://github.com/kahleeeb3/roboclaw_driver.git \
        /opt/src_ws/src/roboclaw_driver \
    && bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
        && cd /opt/src_ws \
        && colcon build --install-base /ros_ws/install" \
    && rm -rf /opt/src_ws/build /opt/src_ws/log

# Modify the ROS entrypoint
RUN cat > /ros_entrypoint.sh <<'EOF'
#!/bin/bash
set -e
source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f "/ros_ws/install/setup.bash" ]; then
  source "/ros_ws/install/setup.bash"
fi
exec "$@"
EOF

WORKDIR /ros_ws