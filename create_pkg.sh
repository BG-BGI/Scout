#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "Error: .env not found at $ENV_FILE" >&2
  exit 1
fi

source "$ENV_FILE"

: "${DOCKER_IMAGE:?DOCKER_IMAGE must be set in .env}"
: "${PACKAGE_NAME:?PACKAGE_NAME must be set in .env}"
TARGET_DIR="$(pwd)"

echo "Creating ROS 2 package '$PACKAGE_NAME'"
echo "  Image : $DOCKER_IMAGE"
echo "  Output: $TARGET_DIR"

docker run --rm \
  --network host \
  --ipc host \
  --volume "$TARGET_DIR:/output" \
  --workdir /output \
  --env HOST_UID="$(id -u)" \
  --env HOST_GID="$(id -g)" \
  "$DOCKER_IMAGE" \
  bash -c "
    source /opt/ros/\$(ls /opt/ros | head -1)/setup.bash
    ros2 pkg create '$PACKAGE_NAME' \
      --build-type ament_python
    chown -R \$HOST_UID:\$HOST_GID '/output/$PACKAGE_NAME'
  "

echo "Done — package '$PACKAGE_NAME' created in $TARGET_DIR"
