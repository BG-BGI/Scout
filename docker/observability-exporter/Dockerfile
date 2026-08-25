# Prometheus exporter for the Scout stack (compose service
# `observability_exporter`, profile `observability`). Standalone like
# ros-mcp/scout-skills: it only needs the rosbridge websocket plus the
# docker socket, so no ROS/DDS underlay and no librealsense layer.
FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim
RUN uv pip install --system --no-cache docker "prometheus-client>=0.20" "websockets>=13"
COPY *.py /app/
WORKDIR /app
CMD ["python", "exporter.py"]
