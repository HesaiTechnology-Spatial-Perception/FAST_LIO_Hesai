# FAST_LIO_Hesai Agent Playbooks

Platform-neutral agent "skills" that teach an AI agent how to set up and run
FAST-LIO2 on Hesai JT16 / JT128 LiDARs end to end.

These are **generic** Markdown playbooks — not tied to any single AI product.
Any agent (Claude, GPT, or a custom LLM pipeline) can use them: load
the relevant file as context / system prompt and follow the steps. Each file
has a small YAML header (`name`, `description`) for skill systems that read it,
and a plain-Markdown body that any model can follow directly.

## Files

| File | Target | Use |
| --- | --- | --- |
| `run_fastlio2_ros1.md` | ROS 1 Melodic / Noetic (`main` branch) | Build with `catkin_make`, launch with `roslaunch` |
| `run_fastlio2_ros2.md` | ROS 2 Humble (`ROS2` branch) | Build with `colcon`, launch with `ros2 launch` |

Pick the file matching your ROS version (or the checked-out branch).

## What the playbooks cover

A 5-step decision workflow:

1. Detect the local environment (ROS distro, build state, dependencies)
2. Configure and build the workspace if needed
3. Choose a data source:
   - a PCAP capture → convert to a rosbag
   - an existing rosbag → run directly
   - no data → bring up a live sensor
4. Validate config and input (`check_config.py`, `check_input.py`)
5. Run FAST-LIO2 and view the result in RViz

The playbooks reference the tools in this directory's parent (`tools/`):
`check_input.py`, `check_config.py`, and `pcap_to_rosbag/`.

## Using with a generic LLM

These files are plain instructions. To use outside an agent runtime, paste the
relevant playbook into the model's context as a system / developer prompt, then
ask the model to execute the steps in your shell.
