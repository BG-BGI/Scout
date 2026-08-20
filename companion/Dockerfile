# Companion image (ADR-0021): plain apt, deliberately NOT scout:latest — that
# image drags the 13-minute librealsense source build and lacks rtabmap.
# arm64-native under the UTM VM on Apple Silicon.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rtabmap-ros \
        ros-humble-image-transport-plugins \
        ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Base image's /ros_entrypoint.sh (sources /opt/ros/humble) is all we need.
