# Docker ROS 2
This repository hosts files for quickly getting a development environment for ROS 2 working in docker.

## 1. Name Your Package
In the `.env` file, set these values:

```bash
DOCKER_IMAGE="test:latest"
PACKAGE_NAME="my_robot_pkg"
```

## 2. Build Your Docker Container
I have provided a basic docker image. Build it using:

```bash
docker compose build
```

This will create an image with the name specified in the `.env` file.

## 3. Create Your Package
Run this script to create a Python ROS 2 package:

```bash
./create_pkg.sh
```

This will create a new package folder in your current directory with the package name specified in the `.env` file.

## 4. Build Your Workspace
This runs only the `build_package` service which will create a docker volume with all of the workspace files.

```bash
docker compose --profile build run --rm build_package
```

When you need to rebuild the workspace, run this again.

> **Note:** If you are not using WSL, comment out the lines in `docker-compose.yaml` marked with `# only in WSL`.

## 5. Start Your Services

```bash
docker compose up
```

This starts a service that tests your package and one that publishes "Hello World" to the `/chatter` topic. Verify that you can see the message in Foxglove.

## 6. Remove Everything
Kill the running services with `Ctrl+C`. Then remove all containers and volumes using:

```bash
docker compose down -v
```

## 7. Git
Don't forget to set your git username and email:

```bash
git config user.name "Your Name"
git config user.email "your@email.com"
```