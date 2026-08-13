# MCP endpoint for Magnus. Standalone on purpose: ros-mcp only speaks the
# rosbridge websocket, so it must not drag in scout:latest's librealsense layer.
# ros-mcp depends on full opencv-python (not -headless), whose native module
# needs libGL/libglib/libxcb — absent from slim images; --help dies with
# "ImportError: libxcb.so.1" without them.
# Tool is baked at build time so container start needs no network.
FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 libxcb1 libsm6 libxext6 \
    && rm -rf /var/lib/apt/lists/*
RUN uv tool install ros-mcp==3.1.0
ENV PATH="/root/.local/bin:$PATH"
