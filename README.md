# docker-ros2-env

Japanese documentation is available in [README.ja.md](README.ja.md).

## Windows Notes

- Use Git Bash as the recommended shell when working on Windows.
- Before cloning, run `git config --global core.autocrlf false` to keep LF line endings; reclone if files were fetched earlier with CRLF.
- Start Docker Desktop and keep it running before invoking any compose or helper commands.

## Overview

This repository provides a ROS 2 development container that shares your host workspace with the container. Source code, build artifacts, and launches stay in sync while you run `ros2 run` or `ros2 launch` through `docker exec` wrappers.

- Base image: `ros:jazzy-ros-base` (override with `docker compose build --build-arg ROS_DISTRO=humble`).
- `workspace/` is mounted at `/workspaces/host_ws`, so host and container share sources and build outputs.
- The entrypoint creates a user matching the host UID/GID to avoid file ownership issues when entering via helper scripts.
- `scripts/ros2_exec.sh` wraps `ros2 ...` calls so you never have to source setups manually.

## Quick Start

1. **Build the image**
   ```bash
   docker compose build ros2
   ```
   *Switch distros by passing `--build-arg ROS_DISTRO=humble` and updating `.env`.*

2. **Start the container**
   ```bash
   docker compose up -d ros2
   ```

3. **Run ROS 2 commands**
   ```bash
   ./scripts/ros2_exec.sh run demo_nodes_cpp talker
   ./scripts/ros2_exec.sh launch demo_nodes_cpp talker_listener.launch.py
   ```

4. **Open an interactive shell** (for `colcon build`, debugging, etc.)
   ```bash
   ./scripts/container_shell.sh
   ```
   The helper adjusts to the host UID/GID automatically, so mounted files remain owned by the host user.

5. **Stop the container**
   ```bash
   docker compose stop ros2
   ```

## Workspace Layout

- Follow a standard colcon layout under `workspace/src/<package>`.
- Keep generated `build/`, `install/`, and `log/` directories outside version control.
- Launch files and tests should live alongside their packages (e.g., `workspace/src/<package>/launch`).

## Container Configuration

`.env` controls runtime defaults:

- `ROS2_UID` / `ROS2_GID`: UID/GID for the user created by the entrypoint. Leave unset to let helper scripts pick up the host IDs automatically.
- `ROS2_USERNAME` / `ROS2_GROUPNAME` (optional): Override the user/group names mapped to the IDs (default `ros2`).
- `ROS_DISTRO`: Target ROS distro (default `jazzy`).
- `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`: DDS settings.
- `ROS2_CONTAINER_NAME`: Container name used by helper scripts.

For GUI apps such as RViz, grant X11 access on the host (e.g., `xhost +local:`) before launching.

## Preinstalled ROS Packages

Besides the base install, the image includes:

- `ros-${ROS_DISTRO}-demo-nodes-cpp`
- `ros-${ROS_DISTRO}-xacro`
- `ros-${ROS_DISTRO}-rviz2`
- `ros-${ROS_DISTRO}-joy`

Extend `Dockerfile` with additional dependencies as needed, then rebuild with `docker compose build ros2`.

## Common Workflow

- Keep `docker compose up -d ros2` running and use helper scripts from multiple terminals.
- Add system dependencies to `Dockerfile` and Python dependencies to `requirements.txt`; rebuild after changes.
- Build with `colcon build` inside the container; reuse the `install/` directory in subsequent sessions.
- Avoid `docker compose down -v` unless you want to clear the mounted workspace volumes.

## Windows Notes

- Use Git Bash as the recommended shell when working on Windows.
- Before cloning, run `git config --global core.autocrlf false` to keep LF line endings; reclone if files were fetched earlier with CRLF.
- Start Docker Desktop and keep it running before invoking any compose or helper commands.
