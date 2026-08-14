# Task-level MCP endpoint for Magnus (compose service `scout_skills`).
# Standalone like docker/ros-mcp and for the same reason: it only speaks the
# rosbridge websocket, so no ROS, no DDS, no librealsense layer. Deps baked at
# build time so container start needs no network.

# Throwaway stage: ultralytics drags torch (~1.5 GB installed) just to export
# YOLO11n to ONNX. The runtime stage copies the 10 MB .onnx and runs it on
# onnxruntime — torch never reaches the shipped image.
FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim AS yolo-export
# ultralytics imports full opencv-python, whose native module needs libGL/
# libglib/libxcb — absent from slim images (same trap docker/ros-mcp hit).
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 libxcb1 libsm6 libxext6 \
    && rm -rf /var/lib/apt/lists/*
RUN uv pip install --system --no-cache ultralytics onnx
# Downloads yolo11n.pt (~5 MB) and writes yolo11n.onnx beside it — cwd is /,
# so the artifact IS /yolo11n.onnx (an mv onto itself fails the layer).
RUN yolo export model=yolo11n.pt format=onnx imgsz=640 && test -f /yolo11n.onnx

FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim
# numpy/pillow/onnxruntime all ship aarch64 wheels; nothing here compiles.
# dt-apriltags: the AprilTag detector with an actual aarch64 wheel (bundled C
# lib via ctypes). pupil-apriltags ships none — do not swap.
RUN uv pip install --system --no-cache "fastmcp>=2.10,<3" "websockets>=13" numpy pillow onnxruntime dt-apriltags
COPY --from=yolo-export /yolo11n.onnx /app/yolo11n.onnx
COPY *.py /app/
CMD ["python", "/app/server.py"]
