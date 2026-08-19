# Fleet status backend for the webui (compose service `fleet_status`).
# Standalone like docker/ros-mcp and docker/scout-skills: it never touches
# ROS, only the Docker socket and /proc, so no ROS/DDS layer is needed.
FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim
# network-manager gives us the `nmcli` client used to drive the host's
# NetworkManager over D-Bus (system bus socket is bind-mounted at runtime).
RUN apt-get update && apt-get install -y --no-install-recommends network-manager \
    && rm -rf /var/lib/apt/lists/*
RUN uv pip install --system --no-cache docker
COPY server.py /app/server.py
CMD ["python", "/app/server.py"]
