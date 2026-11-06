# Maze Runner

A Husarion ROSbot XL drives itself through a maze in simulation.

- **Robot:** mecanum wheels driven as a differential robot, with an RPLIDAR S3.
- **Localization:** an EKF that matches lines seen by the lidar to the walls of the map.
  With `slam:=true` it builds that map of lines itself instead (EKF SLAM).
- **Planning:** A\* with shortcuts plans a route from the robot to the goal on the map.
- **Control:** an NMPC controller follows the route. A control barrier function, or CBF,
  keeps it clear of walls and obstacles.
- **Stack:** ROS 2 Lyrical, Gazebo Jetty, all C++.

## Contents

- [Setup](#setup)
- [Run](#run)
  - [Planners](#planners)
- [Launch arguments](#launch-arguments)
- [Recording a run](#recording-a-run)
- [Packages](#packages)
- [Code layout](#code-layout)
- [Frames](#frames)
- [Limits](#limits)
- [Adding a real robot](#adding-a-real-robot)
- [License](#license)

## Setup

You only need [pixi](https://pixi.sh). It installs ROS 2 from RoboStack, Gazebo and CasADi
from `pixi.toml`.

```bash
pixi run build   # colcon build
pixi run test    # unit tests
```

## Run

Start the simulation, localization, RViz and the Gazebo GUI:

```bash
pixi shell
source install/setup.bash
ros2 launch maze_runner_gazebo sim.launch.xml
```

The planner plans from the robot to the goal in `config/maze_goal.yaml` and publishes `/plan`.
Set a new goal any time with "2D Goal Pose" in RViz. Then start driving, in a second terminal:

```bash
ros2 run maze_runner_nodes nmpc_cbf_node --ros-args \
  --params-file install/maze_runner_gazebo/share/maze_runner_gazebo/config/maze_goal.yaml \
  -p use_sim_time:=true
```

Or do both at once with `controller:=true`.

### Planners

Pick one with the `planner` launch argument:

| Planner | What it does |
|---|---|
| `astar_shortcut` | A\* on the grid, then line-of-sight shortcuts. The default. |
| `astar` | A\* on the grid, every cell kept. |
| `hybrid_astar` | Forward arcs with a minimum turn radius. Tune it with the `hybrid.*` parameters. |

To time them on a world without Gazebo:

```bash
ros2 run maze_runner_nodes plan_benchmark WORLD MODEL SX SY SYAW GX GY [RES] [INFL] [RUNS]
```

On `maze.sdf` A\* + shortcut plans in about 6 ms and Hybrid A\* in about 100 to 170 ms.

### Gazebo GUI on macOS

The Gazebo server runs headless and the GUI runs as a separate process. If the GUI fails
to load Metal, launch with `gui:=false` and start the GUI yourself:

```bash
gz sim -g --render-engine-gui-api-backend opengl
```

## Launch arguments

| Argument | Default | What it does |
|---|---|---|
| `world` | `worlds/maze.sdf` | World to load. `worlds/test_maze.sdf` is a small test maze. |
| `gui` | `true` | Start the Gazebo GUI. |
| `rviz` | `true` | Start RViz. |
| `localization` | `true` | Start the EKF and planner nodes. |
| `slam` | `false` | Build the line map online (EKF SLAM) instead of loading it from `world`. The map frame starts at the robot's start pose. |
| `controller` | `false` | Start the NMPC node and drive right away. |
| `planner` | `astar_shortcut` | Planner to use. See [Planners](#planners). |
| `goal` | `config/maze_goal.yaml` | Goal to plan to. `config/test_maze_goal.yaml` goes with `test_maze.sdf`. |
| `x`, `y`, `yaw` | maze start | Robot start pose. |
| `bag` | empty | Directory to record a bag to. Empty means no recording. |

## Recording a run

```bash
ros2 launch maze_runner_gazebo sim.launch.xml controller:=true bag:=bags/my_run
```

The bag holds:

- ground truth and EKF pose
- wheel odometry
- commands, both as sent and after rate limiting
- lidar scans and TF
- the plan and the NMPC reference
- NMPC solve times

Stop the launch with Ctrl-C so the bag is closed properly. If it was killed, repair it with:

```bash
ros2 bag reindex bags/my_run -s mcap
```

## Packages

| Package | What it holds |
|---|---|
| `maze_runner_core` | The algorithms, with no ROS, and their unit tests. |
| `maze_runner_nodes` | The ROS nodes. `ekf_lines_node` publishes `ekf/pose` and `map -> odom`, plus the mapped walls on `ekf/landmarks` in SLAM mode. `planner_node` publishes `plan`. `nmpc_cbf_node` follows it and publishes `diff_drive_controller/cmd_vel`. |
| `maze_runner_description` | Robot URDF, ros2_control config and Husarion meshes under Apache-2.0. |
| `maze_runner_gazebo` | Worlds, goals, Gazebo bridge, RViz config and `sim.launch.xml`. |

## Code layout

Both C++ packages are split into the same modules. Each module has its own namespace, and
`src/` and `test/` mirror `include/`.

| Module | Namespace | In `maze_runner_core` | In `maze_runner_nodes` |
|---|---|---|---|
| `localization` | `maze_runner::localization` | Line EKF, line EKF SLAM, RANSAC line extraction, map lines from walls and meshes, odometry deltas | `ekf_lines_node`, and `WorldMapLoader` to read the walls from a world SDF |
| `planning` | `maze_runner::planning` | Occupancy grid, A\* with shortcut, Hybrid A\* | `planner_node`, `plan_benchmark` |
| `control` | `maze_runner::control` | NMPC-CBF with CasADi and IPOPT, path tracker | `nmpc_cbf_node` |
| shared | `maze_runner` | `angles.hpp`, `walls.hpp` | `common/yaw.hpp`, `common/world_map_loader.hpp` |

Two rules keep the code easy to follow:

- Free functions are pure. They only compute a result from their inputs.
- Anything with state or I/O is a class. These are the EKF, the path tracker, the NMPC,
  the map loader and the nodes.

## Frames

Frames follow REP 105:

```
map -> odom -> base_footprint -> base_link
```

## Limits

- **Stale data:** the controller stops if its pose is older than 0.5 s or its scan is
  older than 0.3 s.
- **Unknown obstacles:** the planner only knows the walls in the world file. If an obstacle
  that is not in the map blocks the whole plan, the robot stops in front of it.
- **Odometry calibration:** the wheel odometry multipliers in `controllers.yaml` are tuned
  to how Gazebo approximates mecanum friction. Calibrate them again on a real robot.

## Adding a real robot

There is no `maze_runner_bringup` package yet. The simulation is the only way to run the
stack, so `sim.launch.xml` is the single entry point.

When the real ROSbot XL comes in, add `maze_runner_bringup`:

1. Move the node launch, `maze_goal.yaml` and the RViz config into it.
2. Make `sim.launch.xml` include the bringup launch.
3. Add a real-robot launch that starts the Husarion drivers and the lidar, and includes the
   same bringup launch.

## License

The code is under the MIT License. See [LICENSE](LICENSE).

The robot and lidar meshes in `maze_runner_description/meshes` come from Husarion and are
under the Apache License 2.0. See
[LICENSE.husarion](maze_runner_description/meshes/LICENSE.husarion).
