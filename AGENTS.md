# Repository Guidelines

## Project Structure & Module Organization
- `camera_ws/` is a ROS2 workspace; build artifacts live under `camera_ws/build`, `camera_ws/install`, and `camera_ws/log` after compilation.
- `camera_ws/src/GraspEveryThing/` is the core project.
- `camera_ws/src/GraspEveryThing/software/` holds Python scripts, configs (for example `config.csv`), and model weights (`weights.pth`).
- `camera_ws/src/GraspEveryThing/hardware/` contains CAD assets for gripper fabrication.
- `camera_ws/src/GraspEveryThing/figures/` and `camera_ws/src/GraspEveryThing/videos/` store media assets for documentation.
- `document/` keeps research notes and long-form setup guidance.

## Build, Test, and Development Commands
- Build the ROS2 workspace (from `camera_ws/`):
  ```sh
  colcon build
  ```
- Source the workspace before running ROS nodes:
  ```sh
  source /home/hayashi/worksp/camera_ws/install/setup.bash
  ```
- Stream tactile video (see `GraspEveryThing` README for context):
  ```sh
  python3 camera_ws/src/GraspEveryThing/software/get_tactile_video.py
  ```

## Coding Style & Naming Conventions
- Python uses 4-space indentation and `snake_case` for variables and functions.
- Keep file and directory names lowercase with underscores (for example `get_stream_from_url.py`).
- Prefer small, single-purpose scripts in `software/` with clear CLI usage at the top when relevant.

## Testing Guidelines
- There are no automated tests in this repo today.
- Validate changes by running the relevant script or ROS launch file and verifying camera stream output.
- If you add tests, colocate them with the module (for example `software/tests/`) and document how to run them here.

## Commit & Pull Request Guidelines
- Git history shows short, plain-English subjects; follow the same pattern (for example, "add imaging driver notes").
- Include context in PR descriptions: hardware used, ROS version, and steps to reproduce the change.
- Link related notes or setup docs in `document/` when behavior depends on environment configuration.

## Language & Documentation
- Explain changes in Japanese in PR descriptions and local notes; add brief English terms only when needed for tool names or error messages.
- Keep instructions concise and action-oriented, matching the style of the existing notes.

## sync obsidian workspace to Google drive
- After editing any Markdown (`.md`) files, make sure to run Sync.
- Synchronize with `$ rclone copy ./worksp/document gdrive:obsidian-backup --verbose`

## Security & Configuration Tips
- Do not commit credentials, IPs, or private network details; keep them in local notes.
- When scripts depend on device paths or hostnames, make them configurable (env vars or small config files).
