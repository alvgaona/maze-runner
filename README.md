# Maze Runner: A Differential Mobile Robot for Maze Navigation

## Prerequisites

- MATLAB (>=R2023b)
- [CasAdi for MATLAB](https://github.com/casadi/casadi/releases/tag/3.7.2)
- [Apolo](https://github.com/mhernando/Apolo) for any scripts under the [apolo](/apolo)
folder, as these are integrated with the simulator.

## Usage

Maze Runnner can be simulated with a Apolo (3D simulator) or standalone MATLAB with fewer capabilities.

### Apolo and MATLAB

The very first thing to do is to pull up Apolo.
Opening the desired world is necessary for the MATLAB scripts to integrate with it.
The recommended Apolo world is [`MazeRunnerSimple.mwf`](apolo/world/MazeRunnerSimple.mwf).
A maze (garden) and the mobile robot should be visible and placed correctly.

At this point, MATLAB is the second part of this.
All scripts and algorithms that interact directly with Apolo are in the `apolo/` directory of the project.
It leverages other modules, e.g., control, dynamics, planning; these modules can be used without Apolo.
The entry point of this simulation should be [`apolo_full_lines.m`](apolo/apolo_full_lines.m), which connects
everything and even shows visual plots related to the simulation.
It has several parameters but most of the default ones should work out of the box.

### Standalone MATLAB

Standalone [scripts](standalone/) are simulations based entirely on MATLAB.
These scripts are essentially built to tests algorithms and integrations without relying on Apolo.
The nonlinear MPC is mostly implemented in CasAdi so the folder downloaded from
[casadi/casadi](https://github.com/casadi/casadi) release needs to be added to the MATLAB path.

