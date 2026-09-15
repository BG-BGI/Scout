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
# AprilTag detection runs in apriltag_ros (robot container) — this image only
# consumes /detections over rosbridge, so no detector library here. (dt-
# apriltags was tried and dropped: multi-family silently detects nothing and
# Detector destruction corrupts malloc.)
RUN uv pip install --system --no-cache "fastmcp>=2.10,<3" "websockets>=13" numpy pillow onnxruntime pyyaml
# Schindler RBL SDK (ADR-0030), pinned by SHA like the roboclaw fork. Private
# BG-BGI repo, so the build takes a BuildKit secret `gh_token` (a GitHub token
# with repo read). Local build: GH_TOKEN=$(gh auth token) docker compose build
# scout_skills. Installs from the archive tarball — no git in the slim image;
# the token is read at RUN time from /run/secrets, never recorded in a layer.
ARG SCHINDLER_RBL_REF=ca6aa9b50c7ff99d0222af02504b3a4bc18bc8d5
RUN --mount=type=secret,id=gh_token \
    uv pip install --system --no-cache \
    "schindler-rbl @ https://x-access-token:$(cat /run/secrets/gh_token)@github.com/BG-BGI/schindler-rbl/archive/${SCHINDLER_RBL_REF}.tar.gz"
COPY --from=yolo-export /yolo11n.onnx /app/yolo11n.onnx
COPY *.py /app/
CMD ["python", "/app/server.py"]
