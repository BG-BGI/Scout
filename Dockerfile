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
# Deliberately NOT /ros_ws/install: that path is a named volume at runtime, and a
# volume only seeds from the image while it is still empty. Anything baked there
# after the volume exists is silently invisible, and later image rebuilds can never
# reach it. /ros_ws/install belongs to colcon as the output for our own packages.
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

# Robot description publishing. Deliberately after the librealsense build: adding
# these to the apt layer at the top invalidates its cache and costs a full
# librealsense rebuild (~13 min on a Pi 5).
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Modify the ROS entrypoint
RUN cat > /ros_entrypoint.sh <<'EOF'
#!/bin/bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
# Overlays in ascending precedence: image-baked source packages first, then the colcon
# workspace from the ros_ws_install volume, so a locally built package wins. Adding a
# source package means adding a RUN above, never touching this list.
for overlay in "$OVERLAY/install" /ros_ws/install; do
  if [ -f "$overlay/setup.bash" ]; then
    source "$overlay/setup.bash"
  fi
done
exec "$@"
EOF

WORKDIR /ros_ws