clear;
close all;
clc;

set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');

%% Map Definition

% Map is defined in Hesse form: (d,α).
% d: perpendicular distance to the origin.
% α: angle of that vector whose norm is the perpendicular distance.
load('map/garden_lines.mat')

num_walls = size(map_lines,1);

%% Vehicle and Simulation Parameters
dt = 0.02;                         % Simulation/EKF time step [s] - 50 Hz
control_dt = 0.05;                 % Control time step [s] - 20 Hz
sim_time = 100;                    % Total simulation time [s]
num_steps = sim_time / dt;         % Total simulation steps
control_rate = control_dt / dt;    % Control updates every N steps

%% Initialize storage
true_trajectory = zeros(3, num_steps);      % [x; y; theta]
control_history = zeros(2, num_steps);      % [v; omega]
estimated_trajectory = zeros(3, num_steps);
covariance_history = zeros(3, num_steps);

% Initial state
true_trajectory(:, 1) = [1; 1; 0];          % Start at (1,1) heading east

%% EKF Initialization
x = true_trajectory(:, 1);  % Initial state
P = diag([0.1 0.1 deg2rad(1)].^2);    % Initial state covariance

% Process noise covariance for odometry [Δd, Δβ]
process_noise_d = 9.5003e-05;      % Distance increment noise [m]
process_noise_beta = 3.9080e-05;   % Heading increment noise [rad]
Q = diag([process_noise_d^2, process_noise_beta^2]);

% Create EKF instance
chi2_threshold = 9.21;  % 99% confidence for 2 DOF
ekf = EKFLines(x, P, map_lines, Q, chi2_threshold);

estimated_trajectory(:, 1) = x;
covariance_history(:, 1) = sqrt(diag(P));

%% Set up laser sensor
scanner = LMSScanner('LMS100', 'MaxRange', 10, 'NoiseStd', 0.018085189925279);

%% Main Simulation Loop (50 Hz)
nominal_velocity = 0.5; % Nominal speed [m/s]
u_current = [nominal_velocity; 0]; % Initialize control

for k = 1:num_steps-1
    % Update control at 20 Hz (every control_rate steps)
    if mod(k-1, round(control_rate)) == 0
        if mod(k, 500) < 375  % Adjusted for 50 Hz (was 100/75 at 10 Hz)
            u_current = [nominal_velocity; 0];
        else
            u_current = [nominal_velocity; deg2rad(30)];
        end
    end

    control_history(:, k) = u_current;

    % Propagate true state using ode45 with unicycle dynamics
    odefun = @(t, x) unicycle(x, u_current);
    [~, x_traj] = ode45(odefun, [0, dt], true_trajectory(:, k));
    true_trajectory(:, k+1) = x_traj(end, :)';

    % Simulate odometry sensors
    state_delta = true_trajectory(:, k+1) - true_trajectory(:, k);
    actual_delta_d = norm(state_delta(1:2));
    actual_delta_theta = state_delta(3);

    % Add odometry measurement noise
    noisy_delta_d = actual_delta_d + process_noise_d * sqrt(dt) * randn;
    noisy_delta_theta = actual_delta_theta + process_noise_beta * sqrt(dt) * randn;

    % EKF Prediction with noisy odometry [Δd, Δβ]
    ekf.predict(noisy_delta_d, noisy_delta_theta);

    % EKF Update with LiDAR
    scan = scanner.scan(true_trajectory(:, k+1), map_lines);

    % Extract lines from scan using RANSAC
    lines_observed = ransac_lines(scan, 0.015, 3);

    ekf.update(lines_observed);

    % Store EKF estimates
    estimated_trajectory(:, k+1) = ekf.x;
    covariance_history(:, k+1) = sqrt(diag(ekf.P)); % Store standard deviations
end

%% Compute Estimation Errors
position_error_ekf = sqrt(sum((estimated_trajectory(1:2, :) - true_trajectory(1:2, :)).^2, 1));
heading_error_ekf = estimated_trajectory(3, :) - true_trajectory(3, :);
heading_error_ekf = atan2(sin(heading_error_ekf), cos(heading_error_ekf));  % Normalize to [-pi, pi]
time_vector = (0:num_steps-1) * dt;

%% Visualization (ignore logic)
fig_animation = figure('Name', 'EKF Animation');

subplot(1, 3, 1);
hold on; grid on; axis equal;

% Draw map walls once
for w = 1:num_walls
    alpha = map_lines(w, 1);
    d = map_lines(w, 2);
    n = [cos(alpha), sin(alpha)];
    p1 = d * n + 20 * [-n(2), n(1)];
    p2 = d * n - 20 * [-n(2), n(1)];
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'k-', 'LineWidth', 2);
end

h_true_traj = plot(true_trajectory(1, 1), true_trajectory(2, 1), 'b-', 'LineWidth', 2);
h_est_traj = plot(estimated_trajectory(1, 1), estimated_trajectory(2, 1), 'r--', 'LineWidth', 2);
h_true_robot = plot(true_trajectory(1, 1), true_trajectory(2, 1), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
h_est_robot = plot(estimated_trajectory(1, 1), estimated_trajectory(2, 1), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
h_scan_hits = plot(NaN, NaN, 'r.', 'MarkerSize', 4);
h_scan_misses = plot(NaN, NaN, '.', 'Color', [0.5 0 0.5], 'MarkerSize', 4);  % Violet
h_rays_hits = plot(NaN, NaN, 'Color', [1 0.5 0.5], 'LineWidth', 0.5);  % Light red rays
h_rays_misses = plot(NaN, NaN, 'Color', [0.7 0.5 0.7], 'LineWidth', 0.5);  % Light violet rays
h_observed_lines = [];  % Will hold multiple line handles

xlabel('X [m]'); ylabel('Y [m]');
title('Spatial View');

h_map = plot(NaN, NaN, 'k-', 'LineWidth', 2);
h_lidar_rays = plot(NaN, NaN, 'Color', [1 0.5 0.5], 'LineWidth', 0.5);
h_lidar_points = plot(NaN, NaN, 'r.', 'MarkerSize', 4);
h_observed = plot(NaN, NaN, 'g-', 'LineWidth', 2);

xlim([-5 12]); ylim([-5 20]);

subplot(1, 3, 2);
hold on; grid on;
h_error_ekf = plot(time_vector(1), position_error_ekf(1), 'r-', 'LineWidth', 2);
h_heading_error = plot(time_vector(1), rad2deg(heading_error_ekf(1)), 'b-', 'LineWidth', 2);

xlabel('Time [s]'); ylabel('State Error');
title('State Error');
legend('Position Error', 'Heading Error', 'Location', 'best', 'AutoUpdate', 'off');
xlim([0 sim_time]); ylim([0 max(0.1) * 1.1]);

subplot(1, 3, 3);
hold on; grid on;
h_cov_x = plot(time_vector(1), covariance_history(1, 1), 'r-', 'LineWidth', 1.5);
h_cov_y = plot(time_vector(1), covariance_history(2, 1), 'g-', 'LineWidth', 1.5);
h_cov_theta = plot(time_vector(1), rad2deg(covariance_history(3, 1)), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Standard Deviation');
title('EKF Uncertainty ($\sigma$)');
legend('$\sigma_x$ [m]', '$\sigma_y$ [m]', '$\sigma_\theta$ [deg]', 'Location', 'best', 'AutoUpdate', 'off');
xlim([0 sim_time]);
ylim([0 max([max(covariance_history(1:2, :)), rad2deg(max(covariance_history(3, :)))]) * 1.1]);

animation_step = 5;
for k = 1:animation_step:num_steps
    subplot(2, 3, [1 4]);

    set(h_true_traj, 'XData', true_trajectory(1, 1:k), 'YData', true_trajectory(2, 1:k));
    set(h_est_traj, 'XData', estimated_trajectory(1, 1:k), 'YData', estimated_trajectory(2, 1:k));

    set(h_true_robot, 'XData', true_trajectory(1, k), 'YData', true_trajectory(2, k));
    set(h_est_robot, 'XData', estimated_trajectory(1, k), 'YData', estimated_trajectory(2, k));
    
    scan = scanner.scan(true_trajectory(:, k), map_lines);
    theta_k = true_trajectory(3, k);
    robot_pos = true_trajectory(1:2, k);

    valid = ~isnan(scan(:, 1));
    invalid = isnan(scan(:, 1));

    % Process hits (valid returns)
    if any(valid)
        scan_xy_hits = scan(valid, 1) .* [cos(scan(valid, 2) + theta_k), sin(scan(valid, 2) + theta_k)];
        scan_world_hits = [robot_pos(1) + scan_xy_hits(:, 1), robot_pos(2) + scan_xy_hits(:, 2)];
        set(h_scan_hits, 'XData', scan_world_hits(:, 1), 'YData', scan_world_hits(:, 2));

        n_rays_hits = sum(valid);
        ray_x_hits = zeros(3 * n_rays_hits, 1);
        ray_y_hits = zeros(3 * n_rays_hits, 1);
        idx_hit = find(valid);
        for i = 1:n_rays_hits
            idx = 3 * (i - 1);
            ray_x_hits(idx + 1) = robot_pos(1);
            ray_y_hits(idx + 1) = robot_pos(2);
            ray_x_hits(idx + 2) = scan_world_hits(i, 1);
            ray_y_hits(idx + 2) = scan_world_hits(i, 2);
            ray_x_hits(idx + 3) = NaN;
            ray_y_hits(idx + 3) = NaN;
        end
        set(h_rays_hits, 'XData', ray_x_hits, 'YData', ray_y_hits);
    else
        set(h_scan_hits, 'XData', NaN, 'YData', NaN);
        set(h_rays_hits, 'XData', NaN, 'YData', NaN);
    end

    % Process misses (invalid returns - draw to max range)
    if any(invalid)
        miss_angles = scan(invalid, 2);
        miss_ranges = ones(sum(invalid), 1) * scanner.max_range;
        scan_xy_misses = miss_ranges .* [cos(miss_angles + theta_k), sin(miss_angles + theta_k)];
        scan_world_misses = [robot_pos(1) + scan_xy_misses(:, 1), robot_pos(2) + scan_xy_misses(:, 2)];
        set(h_scan_misses, 'XData', scan_world_misses(:, 1), 'YData', scan_world_misses(:, 2));

        n_rays_misses = sum(invalid);
        ray_x_misses = zeros(3 * n_rays_misses, 1);
        ray_y_misses = zeros(3 * n_rays_misses, 1);
        for i = 1:n_rays_misses
            idx = 3 * (i - 1);
            ray_x_misses(idx + 1) = robot_pos(1);
            ray_y_misses(idx + 1) = robot_pos(2);
            ray_x_misses(idx + 2) = scan_world_misses(i, 1);
            ray_y_misses(idx + 2) = scan_world_misses(i, 2);
            ray_x_misses(idx + 3) = NaN;
            ray_y_misses(idx + 3) = NaN;
        end
        set(h_rays_misses, 'XData', ray_x_misses, 'YData', ray_y_misses);
    else
        set(h_scan_misses, 'XData', NaN, 'YData', NaN);
        set(h_rays_misses, 'XData', NaN, 'YData', NaN);
    end

    lines_observed = ransac_lines(scan, 0.02, 5);

    if ~isempty(h_observed_lines)
        delete(h_observed_lines(isvalid(h_observed_lines)));
    end
    h_observed_lines = [];

    theta_k = true_trajectory(3, k);
    robot_pos = true_trajectory(1:2, k);
    R = [cos(theta_k) -sin(theta_k); sin(theta_k) cos(theta_k)];

    for j = 1:size(lines_observed, 1)
        alpha_robot = lines_observed(j, 1);
        d_robot = lines_observed(j, 2);

        n_robot = [cos(alpha_robot); sin(alpha_robot)];
        tangent_robot = [-sin(alpha_robot); cos(alpha_robot)];

        center_robot = d_robot * n_robot;
        center_world = robot_pos + R * center_robot;
        tangent_world = R * tangent_robot;

        p1 = center_world + 3 * tangent_world;
        p2 = center_world - 3 * tangent_world;

        h = plot([p1(1) p2(1)], [p1(2) p2(2)], 'g-', 'LineWidth', 2);
        h_observed_lines = [h_observed_lines; h];
    end

    subplot(1, 3, 2);
    set(h_error_ekf, 'XData', time_vector(1:k), 'YData', position_error_ekf(1:k));

    subplot(1, 3, 2);
    set(h_heading_error, 'XData', time_vector(1:k), 'YData', rad2deg(heading_error_ekf(1:k)));

    subplot(1, 3, 3);
    set(h_cov_x, 'XData', time_vector(1:k), 'YData', covariance_history(1, 1:k));
    set(h_cov_y, 'XData', time_vector(1:k), 'YData', covariance_history(2, 1:k));
    set(h_cov_theta, 'XData', time_vector(1:k), 'YData', rad2deg(covariance_history(3, 1:k)));

    drawnow('limitrate');
    pause(0.05);
end

figure(fig_animation);
fprintf('Animation complete!\n');
