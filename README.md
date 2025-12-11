# Snydrone

Snydrone is an LLM-driven cinematographic drone system built on ROS 2 Humble,
PX4 v1.14 (Offboard), and high-fidelity simulation (Isaac Sim + Pegasus / AirSim).

This repo is a **scaffold**: it gives you a ready-to-build ROS 2 workspace with
packages for:

- `snydrone_msgs`: shared message definitions
- `llm_agent`: LLM-based natural language → CinematicShot planner
- `shot_planner`: converts CinematicShot into trajectories
- `vision_tracker`: dummy vision-based target tracker
- `traj_follower`: dummy trajectory follower / PX4 interface
- `sim_bridge`: launch file to start the whole stack in sim
- `dataset_logger`: stub node for logging & future bag/export tools

> Nodes are minimal placeholders so the workspace builds & runs even without
> PX4/Isaac/MAVSDK. You can safely iterate and fill in real logic later.

## Quick start (Linux, ROS 2 Humble)

```bash
# In your Linux machine
mkdir -p ~/snydrone_ws/src
cd ~/snydrone_ws/src
# copy or clone this repo into here, so you have ~/snydrone_ws/src/snydrone

cd ~/snydrone_ws
source /opt/ros/humble/setup.bash
colcon build

# New terminal
cd ~/snydrone_ws
source install/setup.bash

# Launch full stack (sim bridge + all nodes)
ros2 launch sim_bridge simulation.launch.py
