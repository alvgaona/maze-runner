clc; clear; close all;

%% Simulation Parameters
dt = 0.025;                  % Simulation time step [s] - 20 Hz
control_dt = 0.05;           % Control time step [s] - 10 Hz
Tsim = 60;                  % Simulation time [s]
t = 0:dt:Tsim;
num_steps = length(t);
control_rate = control_dt / dt;

%% Environment - Horizontal corridor with parallel walls
corridor_width = 4;         % Corridor width [m]
corridor_length = 12;       % Corridor length [m]

% Define corridor walls as line segments in Hesse form [alpha, d]
% Convert walls to line representation
map_lines = [
    0, corridor_width/2;     % Top wall (horizontal)
    pi, corridor_width/2;    % Bottom wall (horizontal)
];

% Define obstacles as cell array (for LiDAR simulation)
obstacles = {};
obstacles{end+1} = struct('type', 'wall', 'p1', [-1; corridor_width/2], 'p2', [corridor_length; corridor_width/2]);
obstacles{end+1} = struct('type', 'wall', 'p1', [-1; -corridor_width/2], 'p2', [corridor_length; -corridor_width/2]);

% Square obstacle in center of corridor
obstacle_center = [5; 0];   % Center position [x; y]
obstacle_size = 1.0;         % Square side length [m]
obstacles{end+1} = struct('type', 'square', 'center', obstacle_center, 'size', obstacle_size);

%% LiDAR Configuration
scanner = LMSScanner('LMS100', 'MaxRange', 10.0, 'NoiseStd', 0.018085189925279);

%% EKF Configuration
% Process noise covariance for odometry [Δd, Δβ]
process_noise_d = 9.5003e-05;       % Distance increment noise [m]
process_noise_beta = 3.9080e-05;    % Heading increment noise [rad]
Q = diag([process_noise_d^2, process_noise_beta^2]);

% Initial state and covariance
x0_true = [0; 0; 0];  % True initial state
x0_ekf = x0_true;     % EKF starts with true state
P0 = diag([0.1, 0.1, deg2rad(1)].^2);  % Initial uncertainty

% Create EKF instance
chi2_threshold = 9.21;  % 99% confidence for 2 DOF
ekf = EKFLines(x0_ekf, P0, map_lines, Q, chi2_threshold);

%% NMPC+CBF Controller Configuration
controller = NMPCCBFController(...
    'HorizonLength', 12, ...
    'TimeStep', control_dt, ...
    'StateWeights', [10, 10, 1], ...
    'ControlWeights', [1, 1], ...
    'VelocityLimits', [0, 1], ...
    'AngularLimits', [-5, 5], ...
    'SafetyRadius', 0.4, ...
    'AlphaCBF', 0.05, ...
    'ScanDownsample', 5, ...
    'ConstraintRange', 4.0, ...
    'MaxIterations', 100, ...
    'UseSlack', true, ...
    'SlackPenalty', 100, ...
    'FoV', 20);

% Extract parameters
d_safe = controller.d_safe;
v_max = controller.v_max;
omega_min = controller.omega_min;
omega_max = controller.omega_max;

%% Reference Trajectory
% Straight horizontal trajectory through center of corridor
v_ref = 0.8;                 % Reference speed [m/s]
x_ref = v_ref * t;           % Move horizontally at constant speed
y_ref = zeros(size(t));      % Stay centered in corridor
theta_ref = zeros(size(t));  % Point along x-axis (0 degrees)

xref = [x_ref', y_ref', theta_ref'];

%% Initialize Storage
true_trajectory = zeros(3, num_steps);
estimated_trajectory = zeros(3, num_steps);
control_history = zeros(2, num_steps);
covariance_history = zeros(3, num_steps);
scan_history = cell(1, num_steps);

true_trajectory(:, 1) = x0_true;
estimated_trajectory(:, 1) = x0_ekf;
covariance_history(:, 1) = sqrt(diag(P0));

%% Main Simulation Loop
fprintf('Starting NMPC+CBF+EKF simulation...\n');
fprintf('Horizontal corridor: width %.1fm, length %.1fm\n', corridor_width, corridor_length);
fprintf('Obstacle at [%.1f, %.1f], size: %.2fm\n', obstacle_center(1), obstacle_center(2), obstacle_size);
fprintf('Safety radius: %.2fm\n', d_safe);
fprintf('Simulation: 20 Hz, Control: 10 Hz, EKF: 20 Hz\n\n');

x_true = x0_true;
u_current = [0; 0];

for k = 1:num_steps-1
    % Store previous state
    x_prev = x_true;

    %% Control Update
    if mod(k-1, round(control_rate)) == 0
        % Use EKF estimate for control
        x_est = ekf.x;

        % Get LiDAR scan from true position
        scan = scanner.scan(x_true, obstacles);

        % Compute control using estimated state
        u_current = controller.compute(x_est, xref(k:end, :), scan);

        if mod(k, 1) == 0
            fprintf('Time: %.1fs, True: [%.2f, %.2f], Est: [%.2f, %.2f]\n', ...
                t(k), x_true(1), x_true(2), x_est(1), x_est(2));
        end
    end

    control_history(:, k) = u_current;

    %% True State Propagation (50 Hz)
    odefun = @(t, x) unicycle(x, u_current);
    [~, x_traj] = ode45(odefun, [0, dt], x_true);
    x_true = x_traj(end, :)';

    true_trajectory(:, k+1) = x_true;

    %% EKF Prediction (50 Hz)
    % Simulate odometry with noise
    state_delta = x_true - x_prev;
    actual_delta_d = norm(state_delta(1:2));
    actual_delta_theta = state_delta(3);

    noisy_delta_d = actual_delta_d + process_noise_d * sqrt(dt) * randn;
    noisy_delta_theta = actual_delta_theta + process_noise_beta * sqrt(dt) * randn;

    ekf.predict(noisy_delta_d, noisy_delta_theta);

    %% EKF Update with LiDAR (50 Hz)
    scan = scanner.scan(x_true, obstacles);
    scan_history{k} = scan;

    % Extract lines from scan using RANSAC
    lines_observed = ransac_lines(scan, 0.015, 3);

    ekf.update(lines_observed);

    %% Store Results
    estimated_trajectory(:, k+1) = ekf.x;
    covariance_history(:, k+1) = sqrt(diag(ekf.P));
end

fprintf('\nSimulation complete!\n');

%% Compute Errors
position_error = sqrt(sum((true_trajectory(1:2, :) - estimated_trajectory(1:2, :)).^2, 1));
heading_error = estimated_trajectory(3, :) - true_trajectory(3, :);
heading_error = atan2(sin(heading_error), cos(heading_error));

%% Visualization
figure('Position', [50 50 1600 1000]);

%% 1. Corridor view with trajectories
subplot(2,3,1);
hold on; grid on; axis equal;

% Draw corridor walls
plot([-1, corridor_length], [corridor_width/2, corridor_width/2], 'k-', 'LineWidth', 2);
plot([-1, corridor_length], [-corridor_width/2, -corridor_width/2], 'k-', 'LineWidth', 2);

% Draw square obstacle
half_size = obstacle_size / 2;
x_square = obstacle_center(1) + [-half_size, half_size, half_size, -half_size, -half_size];
y_square = obstacle_center(2) + [-half_size, -half_size, half_size, half_size, -half_size];
plot(x_square, y_square, 'r-', 'LineWidth', 2);
fill(x_square, y_square, [1 0.8 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'r', 'LineWidth', 2);

% Plot trajectories
plot(xref(:,1), xref(:,2), 'g--', 'LineWidth', 2);
plot(true_trajectory(1,:), true_trajectory(2,:), 'b-', 'LineWidth', 1.5);
plot(estimated_trajectory(1,:), estimated_trajectory(2,:), 'r--', 'LineWidth', 1.5);
plot(true_trajectory(1,1), true_trajectory(2,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(true_trajectory(1,end), true_trajectory(2,end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
title('Corridor Navigation with EKF', 'Interpreter', 'latex');
legend('Wall', '', 'Obstacle', '', 'Reference', 'True', 'EKF Est.', 'Start', 'End', ...
       'Interpreter', 'latex', 'Location', 'best');
xlim([-1, 13]);
ylim([-3, 3]);

%% 2. Position Error
subplot(2,3,2);
plot(t, position_error, 'r-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Position Error [m]', 'Interpreter', 'latex');
title('Position Estimation Error', 'Interpreter', 'latex');
grid on;
xlim([0, Tsim]);

%% 3. Heading Error
subplot(2,3,3);
plot(t, rad2deg(heading_error), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Heading Error [deg]', 'Interpreter', 'latex');
title('Heading Estimation Error', 'Interpreter', 'latex');
grid on;
xlim([0, Tsim]);

%% 4. Control inputs
subplot(2,3,4);
plot(t(1:end-1), control_history(1, 1:end-1), 'b-', 'LineWidth', 1.5); hold on;
plot(t(1:end-1), control_history(2, 1:end-1), 'r-', 'LineWidth', 1.5);
yline(v_max, 'b--', 'LineWidth', 1);
yline(omega_max, 'r--', 'LineWidth', 1);
yline(omega_min, 'r--', 'LineWidth', 1);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Control Input', 'Interpreter', 'latex');
title('Control Inputs', 'Interpreter', 'latex');
legend('$v$ [m/s]', '$\omega$ [rad/s]', 'Interpreter', 'latex');
grid on;
xlim([0, Tsim]);

%% 5. Minimum distance to obstacles
subplot(2,3,5);
min_distances = zeros(num_steps-1, 1);
for k = 1:num_steps-1
    if ~isempty(scan_history{k})
        scan = scan_history{k};
        valid = ~isnan(scan(:,1));
        if any(valid)
            min_distances(k) = min(scan(valid, 1));
        else
            min_distances(k) = NaN;
        end
    else
        min_distances(k) = NaN;
    end
end
plot(t(1:num_steps-1), min_distances, 'k-', 'LineWidth', 1.5); hold on;
yline(d_safe, 'r--', 'LineWidth', 2);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Distance [m]', 'Interpreter', 'latex');
title('Minimum Distance to Obstacles', 'Interpreter', 'latex');
legend('Min distance', 'Safety threshold', 'Interpreter', 'latex');
grid on;
xlim([0, Tsim]);

%% 6. EKF Uncertainty
subplot(2,3,6);
hold on; grid on;
plot(t, covariance_history(1, :), 'r-', 'LineWidth', 1.5);
plot(t, covariance_history(2, :), 'g-', 'LineWidth', 1.5);
plot(t, rad2deg(covariance_history(3, :)), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Standard Deviation', 'Interpreter', 'latex');
title('EKF Uncertainty ($\sigma$)', 'Interpreter', 'latex');
legend('$\sigma_x$ [m]', '$\sigma_y$ [m]', '$\sigma_\theta$ [deg]', ...
       'Interpreter', 'latex', 'Location', 'best');
xlim([0, Tsim]);

sgtitle('NMPC+CBF with EKF State Estimation', 'Interpreter', 'latex');

%% Statistics
fprintf('\n========== Results ==========\n');
fprintf('Final Position Error:   %.3f m\n', position_error(end));
fprintf('Mean Position Error:    %.3f m\n', mean(position_error));
fprintf('Max Position Error:     %.3f m\n', max(position_error));
fprintf('Final Heading Error:    %.3f deg\n', rad2deg(abs(heading_error(end))));
fprintf('\nFinal EKF Std Dev:\n');
fprintf('  Position (x):         %.4f m\n', covariance_history(1, end));
fprintf('  Position (y):         %.4f m\n', covariance_history(2, end));
fprintf('  Heading (theta):      %.4f deg\n', rad2deg(covariance_history(3, end)));
fprintf('============================\n');
