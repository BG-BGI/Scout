FROM ros:humble-ros-core

# System Dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
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
    python3-colcon-common-extensions \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

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