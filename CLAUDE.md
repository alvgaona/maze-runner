# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is a MATLAB-based robotics guidance and navigation repository (Guiado y Navegación de Robots) containing implementations of vehicle dynamics models, state estimation algorithms, and trajectory tracking controllers.

## Running Code

All scripts are MATLAB `.m` files that can be run directly in MATLAB:

```matlab
% Run any script from MATLAB command window
run('main.m')
run('ekf.m')
run('pid_trajectory.m')
```

Or from the command line:
```bash
matlab -batch "run('main.m')"
```

## Architecture

### Vehicle Dynamics Models

The codebase implements two primary vehicle models:

1. **Ackermann Steering Model** (`ackermann.m:1-55`)
   - Function signature: `x_dot = ackermann(x, u, L)`
   - State: `[x, y, theta]` (position and heading)
   - Control: `[v, phi]` (linear velocity, steering angle)
   - Kinematics: `dtheta/dt = (v/L) * tan(phi)`

2. **Differential Drive / Unicycle Model** (used in `ekf.m`, `dead_reckoning.m`, `pid_trajectory.m`, `nmpc.m`)
   - State: `[x, y, theta]`
   - Control: `[v, omega]` (linear velocity, angular velocity)
   - Kinematics: Euler integration with `x_dot = v*cos(theta)`, `y_dot = v*sin(theta)`, `theta_dot = omega`

Recent commits show a transition from Ackermann to Differential Drive model as the primary vehicle model.

### State Estimation

**Extended Kalman Filter (EKF)** (`ekf.m:1-308`)
- Control input for EKF: `[Δd, Δβ]` (odometry-based: distance increment, heading change)
- Measurements: Bearing angles to fixed beacons (3 beacons defined at lines 56-60)
- Process noise in odometry space (lines 39-42)
- Sensor noise: bearing measurement std = 0.05 rad (line 49)
- Uses midpoint odometry model for prediction (lines 136-142)
- Vectorized measurement updates for all beacons (lines 164-196)

**Line-based EKF** (`line_ekf.m:1-244`)
- Simulates LMS200 laser scanner returns
- RANSAC-based line extraction from point clouds (lines 199-240)
- Data association with map lines in Hesse form `[alpha, d]`
- Update step for each observed line feature

### Trajectory Tracking Controllers

**PID Controller** (`pid_trajectory.m:1-172`)
- Circular reference trajectory with configurable center and radius
- Separate PID loops for linear velocity (distance error) and angular velocity (heading error)
- Gains: `Kp_v=0.5, Ki_v=0.01, Kd_v=0.1` for linear, `Kp_omega=2.0, Ki_omega=0.05, Kd_omega=0.3` for angular
- Saturated control outputs: `v_max=2.0 m/s`, `omega_max=2.0 rad/s`

**NMPC (Nonlinear Model Predictive Control)** (`nmpc.m:1-69`)
- Very short prediction horizon (N=3 steps)
- Uses `fmincon` with SQP algorithm for optimization
- Cost function balances state tracking error (Q weights) and control effort (R weights)
- Receding horizon implementation

### Dead Reckoning

`dead_reckoning.m:1-116` demonstrates pure odometry-based estimation with:
- Systematic biases (3% velocity overestimation, 0.005 rad/s angular bias)
- Random noise (velocity std=0.02 m/s, angular std=0.01 rad/s)
- Shows drift characteristics over 20 seconds

## Key Implementation Details

### Integration Methods
- Most simulations use **Euler integration** with discrete time steps (typically `dt=0.1s` or `dt=0.05s`)
- Ackermann model demo uses `ode45` for continuous-time integration (main.m:34)
- Recent commits show optimization from trapezoidal to Euler integration

### Angle Normalization
EKF implementations consistently normalize angles to `[-pi, pi]` using:
```matlab
theta = atan2(sin(theta), cos(theta))
```
This is critical for innovation computation (ekf.m:181) and state updates (ekf.m:193).

### Noise Modeling
- Process noise: Defined in control/odometry space (e.g., `Q` for `[Δd, Δβ]`)
- Measurement noise: Bearing measurements from beacons (typically 0.05 rad std)
- Line-EKF: Per-line uncertainty from RANSAC fitting (line_ekf.m:234-236)

### Trajectory Types
EKF supports three trajectory types (ekf.m:16-33):
- `'linear'`: Constant velocity, zero rotation
- `'circular'`: Constant radius turn
- `'curve'`: Sinusoidal angular velocity profile

## Coordinate Systems

- World frame: 2D planar with x-y coordinates
- Robot frame: Heading `theta` measured from world x-axis
- Beacon positions: Fixed landmarks in world frame
- Line map (Line-EKF): Hesse normal form `[alpha, d]` where alpha is normal angle, d is distance from origin

## Recent Development Focus

Based on git history:
- Transition from Ackermann to Differential Drive dynamics
- Integration method optimization (Euler vs trapezoidal)
- EKF input changed to use displacement-based odometry `[Δd, Δβ]` rather than velocity commands
