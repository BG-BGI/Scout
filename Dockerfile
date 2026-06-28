FROM ros:humble-ros-core

# System Dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    libusb-1.0-0-dev \
    libssl-dev \
    pkg-config \
    python3-pip \
    python3-dev \
    && pip3 install lgpio \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ROS Packages
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-launch-ros \
    ros-humble-rplidar-ros \
    ros-humble-realsense2-* \
    ros-humble-imu-filter-madgwick \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    python3-colcon-common-extensions \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# librealsense from source, RSUSB backend — D455 IMU without host kernel patching
RUN git clone https://github.com/realsenseai/librealsense.git -b v2.57.7 --depth 1 --recurse-submodules \
    && cd librealsense \
    && mkdir build && cd build \
    && cmake .. \
        -DFORCE_RSUSB_BACKEND=ON \
        -DCMAKE_INSTALL_PREFIX=/opt/ros/humble \
        -DCMAKE_INSTALL_LIBDIR=lib/aarch64-linux-gnu \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_PYTHON_BINDINGS=ON \
        -DBUILD_UNIT_TESTS=OFF \
        -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) \
    && make install \
    && cd / && rm -rf librealsense

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