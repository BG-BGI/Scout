# World-model detector (companion, gate 1 of WORLDMODEL.md).
# FROM ros-base (NOT scout-companion:latest) so it builds independently of the
# rtabmap image and carries only what a rclpy+tf2 node needs. amd64 native on
# the Debian box; arm64 works too (all wheels below are multi-arch).

# Throwaway stage: ultralytics drags torch (~1.5 GB) only to export YOLO11n to
# ONNX — identical trick to docker/scout-skills/Dockerfile. The runtime stage
# copies the ~10 MB .onnx and runs it on onnxruntime; torch never ships.
FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim AS yolo-export
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0 libxcb1 libsm6 libxext6 \
    && rm -rf /var/lib/apt/lists/*
RUN uv pip install --system --no-cache ultralytics onnx
RUN yolo export model=yolo11n.pt format=onnx imgsz=640 && test -f /yolo11n.onnx

FROM ros:humble-ros-base
# rclpy, tf2_ros, sensor_msgs/std_msgs are in ros-base. onnxruntime/numpy/
# pillow are the only extras. ros-base has python3 but NO pip, so install
# python3-pip via apt first. Base is Ubuntu 22.04 (jammy): pip 22.x, no PEP-668
# marker, so plain `pip3 install` works and --break-system-packages would error.
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip \
    && pip3 install --no-cache-dir onnxruntime numpy pillow \
    && rm -rf /var/lib/apt/lists/*
COPY --from=yolo-export /yolo11n.onnx /app/yolo11n.onnx
COPY detect.py detector_node.py /app/
WORKDIR /app
# Source ROS, then run the node (rclpy script needs no colcon package).
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "/app/detector_node.py"]
