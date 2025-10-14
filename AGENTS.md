# Repository Guidelines

## Project Structure & Module Organization
Keep the repo aligned with the colcon workspace pattern so rebuilds stay predictable. Runtime entrypoints live in `docker/` (e.g., `ros2_entrypoint.sh`), POSIX-safe helpers in `scripts/`, and ROS 2 packages under `workspace/src/<package>`. Generated `build/`, `install/`, and `log/` directories remain untracked. Launch artifacts and tests stay close to the owning package, typically under `workspace/src/<package>/launch` and `workspace/src/<package>/test`.

## Build, Test, and Development Commands
Run `docker compose build ros2` after editing the Dockerfile or `requirements.txt`; pass `--build-arg ROS_DISTRO=humble` to target another distro and update `.env` in tandem. Start the container with `docker compose up -d ros2`, and reuse it from any terminal. `./scripts/container_shell.sh` drops you into a shell with the ROS environment loaded while impersonating the host UID/GID so shared files keep their ownership. Use `./scripts/ros2_exec.sh run <pkg> <verb>` for one-off ROS CLI calls, e.g., `./scripts/ros2_exec.sh run demo_nodes_cpp talker`.

## Coding Style & Naming Conventions
Author shell scripts with `#!/usr/bin/env bash`, enable `set -euo pipefail`, indent two spaces, and quote expansions. Compose YAML and `.env` keys stay lowercase and mirrored across files, accompanied by comments for defaults. ROS 2 packages follow snake_case directories, standard `ament` layout, and use `ament_lint_auto` plus `ament_clang_format` for C++ formatting where applicable.

## Testing Guidelines
Always run `colcon build` first so generated interfaces exist before tests. Use `colcon test --packages-select <package>` for tighter loops, and name tests descriptively (`test_talker.py`, `test_listener.cpp`, `test_bridge.launch.py`). Capture any manual checks in pull requests if automated coverage cannot demonstrate behavior.

## Commit & Pull Request Guidelines
Use Conventional Commit prefixes (`feat:`, `fix:`, `docs:`) with imperative subjects under 72 characters. Group related configuration edits across `.env`, Dockerfile, and Compose, and call out required follow-up actions such as rebuilding containers. Pull requests should explain motivation, list the executed commands (build, test, runtime), and link issues or design notes.

## Container & Environment Tips
`.env` tracks runtime knobs: `ROS2_UID` / `ROS2_GID` default to the host IDs and drive the user that `ros2_entrypoint.sh` creates at startup; override `ROS2_USERNAME` / `ROS2_GROUPNAME` if you need explicit names. The entrypoint writes resolved IDs to `/run/ros2_env/active_*` for helper scripts—leave those files untouched. Core ROS extras (`ros-${ROS_DISTRO}-xacro`, `ros-${ROS_DISTRO}-rviz2`, `ros-${ROS_DISTRO}-joy`, `ros-${ROS_DISTRO}-demo-nodes-cpp`) are preinstalled; append additional apt packages to the Dockerfile to keep images reproducible. Add Python dependencies to `requirements.txt` and rebuild to bake them into the image.
