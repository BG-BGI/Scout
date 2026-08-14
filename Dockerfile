FROM ros:humble-ros-core

# System Dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    pkg-config \
    libboost-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    python3-dev \
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
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-diagnostic-updater \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------------------------
# Shared overlay for every package built from source into the image.
#
# All of them install into ONE base ($OVERLAY/install), so the entrypoint needs a
# single source line no matter how many get added. colcon merges into an existing
# install base without orphaning what is already there, so each package still gets
# its own RUN layer and caches independently.
#
# Deliberately under $OVERLAY (not a separate /ros_ws/install tree). Compose mounts
# a named volume on /opt/overlay/install so build_package persists; an empty volume
# seeds from this image layer once.
# --------------------------------------------------------------------------------
ENV OVERLAY=/opt/overlay

RUN cat > /usr/local/bin/build-overlay <<'EOF'
#!/bin/bash
# build-overlay <colcon args...> — build $OVERLAY/src into the shared overlay.
# The overlay itself is sourced first when it exists, so a package added in a later
# layer may depend on one installed by an earlier layer.
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f "$OVERLAY/install/setup.bash" ]; then
  source "$OVERLAY/install/setup.bash"
fi
cd "$OVERLAY"
colcon build --install-base "$OVERLAY/install" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
rm -rf "$OVERLAY/build" "$OVERLAY/log"
EOF
RUN chmod +x /usr/local/bin/build-overlay && mkdir -p "$OVERLAY/src"

# roboclaw_driver from source
RUN git clone --depth 1 https://github.com/kahleeeb3/roboclaw_driver.git \
        "$OVERLAY/src/roboclaw_driver" \
    && build-overlay --packages-up-to roboclaw_driver \
    && rm -rf "$OVERLAY/src/roboclaw_driver"

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

# realsense-ros wrapper. The version must track librealsense: wrapper 4.X.Y pairs with
# lib 2.X.Y, so 4.57.7 goes with the v2.57.7 built above. Bump both together.
# --packages-up-to skips realsense2_description, which needs xacro to render its URDF.
RUN git clone --depth 1 -b 4.57.7 https://github.com/IntelRealSense/realsense-ros.git \
        "$OVERLAY/src/realsense-ros" \
    && build-overlay --packages-up-to realsense2_camera \
    && rm -rf "$OVERLAY/src/realsense-ros"

# RPLIDAR driver, from source rather than the ros-humble-rplidar-ros deb. The deb would
# most likely drive the A2-family unit currently attached; the reason for source is that
# it ships SDK 2.1.0 and supports models the deb does not — the deb's description stops
# at "A1/A2/A3/S1/S2/S3/T1" and it ships no rplidar_c1_launch.py, while the ros2 branch
# does, even though BOTH call themselves 2.1.4 (the deb is just built from an older
# commit, so the version string cannot distinguish them). That keeps the image valid if
# the scanner is ever swapped. Trade-off: this tracks the `ros2` branch and so is NOT
# pinned to a commit. Needs no extra apt packages (std_srvs is already present), so it
# sits with the other source builds without disturbing the librealsense cache above.
RUN git clone --depth 1 -b ros2 https://github.com/Slamtec/rplidar_ros.git \
        "$OVERLAY/src/rplidar_ros" \
    && build-overlay --packages-up-to rplidar_ros \
    && rm -rf "$OVERLAY/src/rplidar_ros"

# Robot description publishing, the odometry EKF, SLAM, and navigation. Deliberately
# after the librealsense build: adding these to the apt layer at the top invalidates
# its cache and costs a full librealsense rebuild (~13 min on a Pi 5).
#
# slam-toolbox is expensive: 304 packages and ~866 MiB, because its rviz plugin is a
# hard `find_package(rviz_common REQUIRED)` so the whole rviz/Qt/OGRE stack comes
# along. Building from source would need the rviz *dev* packages instead, which is
# strictly worse, so apt is the cheaper path despite the size.
#
# navigation2 then costs only ~306 MiB on top rather than another ~866, because
# slam-toolbox already dragged in the rviz/Qt/OGRE and nav2_map_server chain. Keep it
# in this same RUN so the two share one apt cache layer. nav2-bringup is what supplies
# navigation_launch.py, which the compose `nav2` service launches directly.
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-map-msgs \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    ros-humble-rosbridge-suite \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# apriltag_ros (christianrauch's ROS 2 port of the official AprilRobotics
# wrapper): continuous AprilTag detection as a native node — /detections +
# a TF frame per tag. Own RUN layer so adding it never invalidates the big
# apt layer above.
RUN apt-get update && apt-get install -y \
    ros-humble-apriltag-ros \
    ros-humble-topic-tools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# explore_lite (frontier exploration) — no Humble apt package; build into $OVERLAY.
# Repo is a multi-package workspace; only lift explore + explore_lite_msgs into src.
# Wipe ros_overlay_install after rebuild so the volume re-seeds with this package.
RUN git clone --depth 1 https://github.com/robo-friends/m-explore-ros2.git /tmp/m-explore-ros2 \
    && mv /tmp/m-explore-ros2/explore /tmp/m-explore-ros2/explore_lite_msgs "$OVERLAY/src/" \
    && rm -rf /tmp/m-explore-ros2 \
    && build-overlay --packages-up-to explore_lite \
    && rm -rf "$OVERLAY/src/explore" "$OVERLAY/src/explore_lite_msgs"

# Modify the ROS entrypoint
RUN cat > /ros_entrypoint.sh <<'EOF'
#!/bin/bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
# Single install tree: image-baked forks and locally built Scout packages both live
# under $OVERLAY/install. Adding a source package means adding a RUN above.
if [ -f "$OVERLAY/install/setup.bash" ]; then
  source "$OVERLAY/install/setup.bash"
fi
exec "$@"
EOF

WORKDIR /ros_ws