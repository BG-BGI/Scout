# Observability MCP endpoint for Claude (compose service
# `observability_mcp`, profile `observability`). Standalone like
# ros-mcp/scout-skills: only needs the docker socket + rosbridge websocket,
# so no ROS/DDS underlay of its own -- `ros2` CLI calls exec into the
# already-running `robot` container instead.
FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim
RUN uv pip install --system --no-cache "fastmcp>=2.10,<3" docker "websockets>=13"
COPY *.py /app/
WORKDIR /app
CMD ["python", "server.py"]
