# Maze Runner

A Husarion ROSbot XL (mecanum wheels driven as a differential robot, RPLIDAR S3) that
localizes with a line-based EKF and follows a route through a maze with an NMPC controller
whose control barrier function (CBF) keeps it clear of walls and obstacles.
ROS 2 Lyrical + Gazebo (Jetty), all C++.

## Packages

| Package | What it holds |
|---|---|
| `maze_runner_core` | Algorithms, no ROS. Unit tests. |
| `maze_runner_nodes` | `ekf_lines_node` (publishes `ekf/pose` and `map -> odom`) and `nmpc_cbf_node` (publishes `diff_drive_controller/cmd_vel`). |
| `maze_runner_description` | Robot URDF, ros2_control config, meshes (Husarion, Apache-2.0). |
| `maze_runner_gazebo` | Worlds (`maze.sdf` from the maze STL, `test_maze.sdf`), route, bridge, RViz config, `sim.launch.xml`. |

The C++ packages are split into the same modules, each with its own namespace; `src/`
and `test/` mirror `include/`:

| Module | `maze_runner_core` | `maze_runner_nodes` |
|---|---|---|
| `localization` (`maze_runner::localization`) | line EKF, RANSAC line extraction, map lines from walls/meshes, odometry deltas | `ekf_lines_node`, `WorldMapLoader` (reads walls from a world SDF) |
| `control` (`maze_runner::control`) | NMPC-CBF (CasADi + IPOPT), path tracker | `nmpc_cbf_node` |
| shared | `angles.hpp` | `common/yaw.hpp` |

Free functions are pure. Anything with state or I/O is a class: the EKF, path tracker,
NMPC, map loader and the nodes.

Frames follow REP 105: `map -> odom -> base_footprint -> base_link`.

## Setup

Needs [pixi](https://pixi.sh); everything else (ROS 2 from RoboStack, Gazebo, CasADi) comes
from `pixi.toml`.

```bash
pixi run build   # colcon build
pixi run test    # unit tests
```

## Run

```bash
pixi shell
source install/setup.bash
ros2 launch maze_runner_gazebo sim.launch.xml            # sim, localization, RViz, Gazebo GUI
ros2 run maze_runner_nodes nmpc_cbf_node --ros-args \
  --params-file install/maze_runner_gazebo/share/maze_runner_gazebo/config/maze_path.yaml \
  -p use_sim_time:=true                                  # start driving
```

The Gazebo server runs headless and the GUI as a separate process (`gz sim -g`). On macOS,
if the GUI fails to load Metal, start it yourself with
`gz sim -g --render-engine-gui-api-backend opengl` and launch with `gui:=false`.

Launch arguments: `world`, `gui`, `rviz`, `localization`, `controller` (start the NMPC from
the launch), `path`, start pose `x y yaw`, and `bag:=<dir>` to record a run.

## Recording a run

`bag:=bags/<name>` records ground truth, EKF pose, odometry, commands (sent and rate-limited),
scans, TF, the reference path and NMPC solve times. Stop the launch with Ctrl-C so the bag is
closed properly (or run `ros2 bag reindex <dir> -s mcap` if it was killed).

## Notes

- The controller stops if its pose is older than 0.5 s or its scan older than 0.3 s.
- A route fully blocked by an obstacle makes the robot stop in front of it: going around
  needs a planner, which this stack does not have.
- Wheel odometry multipliers in `controllers.yaml` are calibrated to Gazebo's mecanum
  friction approximation; calibrate again on a real robot.
