clc; clear; close all;

set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');

%% Load Reference Trajectory and Map
trajectories = load('trayectorias.mat');
traj = trajectories.trajFull;

load('map/garden_lines.mat', 'map_lines');

num_walls = size(map_lines, 1);

%% Simulation Parameters
dt = 0.04;                  % Simulation/EKF time step [s]
control_dt = 0.1;          % Control time step [s]
control_rate = control_dt / dt;

%% LiDAR Configuration
scanner = LMSScanner('LMS100', 'MaxRange', 10.0, 'NoiseStd', 0.018085189925279);

%% EKF Configuration
% Process noise covariance for odometry [Δd, Δβ]
process_noise_d = 9.5003e-05;       % Distance increment noise [m]
process_noise_beta = 3.9080e-05;    % Heading increment noise [rad]
Q = diag([process_noise_d^2, process_noise_beta^2]);

% Initial state
x0_true = traj(1, :)';        % Start at first point of trajectory
x0_ekf = x0_true + [0.1; 0.1; deg2rad(1)]; % Initial estimate with small error
P0 = diag([0.1, 0.1, deg2rad(1)].^2);      % Initial uncertainty

% Create EKF instance
chi2_threshold = 9.21;  % 99% confidence for 2 DOF
ekf = EKFLines(x0_ekf, P0, map_lines, Q, chi2_threshold);

%% NMPC+CBF Controller Configuration
controller = NMPCCBFController(...
    'HorizonLength', 20, ...
    'TimeStep', control_dt, ...
    'StateWeights', [500, 500, 10], ...
    'ControlWeights', [1, 1], ...
    'VelocityLimits', [0, 1], ...
    'AngularLimits', [-2, 2], ...
    'SafetyRadius', 0.5, ...
    'AlphaCBF', 0.1, ...
    'ScanDownsample', 100, ...
    'ConstraintRange', 3.0, ...
    'MaxIterations', 100, ...
    'UseSlack', true, ...
    'SlackPenalty', 100);

% Extract parameters
d_safe = controller.d_safe;
v_max = controller.v_max;
omega_min = controller.omega_min;
omega_max = controller.omega_max;

% Forward-Gated Path Progression
waypoint_threshold = 0.1;  % Advance to next waypoint when within this distance [m]
search_window = 200;       % Max points to search ahead (prevents jumps across loops)
lookahead_dist = 0.5;      % Always track at least this far ahead [m]

%% Interpolate Reference Trajectory to Simulation Rate
% Original trajectory has 84 points, interpolate to finer resolution
traj_original_time = linspace(0, 1, size(traj, 1));  % Normalized time

% Estimate total simulation time based on trajectory length
traj_length = 0;
for i = 1:size(traj, 1)-1
    traj_length = traj_length + norm(traj(i+1, 1:2) - traj(i, 1:2));
end
avg_speed = 1;  % Assume average speed [m/s]
Tsim = traj_length / avg_speed;

% Create time vector at simulation rate
t = 0:dt:Tsim;
num_steps = length(t);
t_normalized = linspace(0, 1, num_steps);

% Interpolate trajectory
xref = zeros(num_steps, 3);
xref(:, 1) = interp1(traj_original_time, traj(:, 1), t_normalized, 'pchip');
xref(:, 2) = interp1(traj_original_time, traj(:, 2), t_normalized, 'pchip');
xref(:, 3) = interp1(traj_original_time, traj(:, 3), t_normalized, 'pchip');

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
fprintf('Starting NMPC+CBF+EKF simulation with Line Features...\n');
fprintf('Reference trajectory: %d points, %.1f m length\n', size(traj, 1), traj_length);
fprintf('Map walls: %d\n', num_walls);
fprintf('Simulation time: %.1f s\n', Tsim);
fprintf('Simulation: %d Hz, Control: %d Hz, EKF: %d Hz\n\n', 1/dt, 1/control_dt, 1/dt);

x_true = x0_true;
u_current = [0; 0];

% Initialize ref_idx to closest waypoint to initial EKF estimate
distances = vecnorm(xref(:,1:2) - x0_ekf(1:2)', 2, 2);
[~, ref_idx] = min(distances);

for k = 1:num_steps-1
    % Store previous state
    x_prev = x_true;

    %% Control Update (20 Hz)
    if mod(k-1, round(control_rate)) == 0
        % Use EKF estimate for control
        x_est = ekf.x;

        % Get LiDAR scan for CBF (obstacle avoidance)
        scan = scanner.scan(x_true, map_lines);

        % Find closest point within search window (prevents jumps across loops)
        end_idx = min(ref_idx + search_window, size(xref, 1));
        distances = vecnorm(xref(ref_idx:end_idx, 1:2) - x_est(1:2)', 2, 2);
        [~, offset] = min(distances);
        ref_idx = ref_idx + offset - 1;

        % Advance waypoint when robot is close enough
        while ref_idx < size(xref, 1) && norm(x_est(1:2) - xref(ref_idx, 1:2)') < waypoint_threshold
            ref_idx = ref_idx + 1;
        end

        % Add lookahead distance to ensure we always track ahead
        track_idx = ref_idx;
        accumulated_dist = 0;
        while track_idx < size(xref, 1) && accumulated_dist < lookahead_dist
            accumulated_dist = accumulated_dist + norm(xref(track_idx+1, 1:2) - xref(track_idx, 1:2));
            track_idx = track_idx + 1;
        end

        % Compute control using estimated state
        u_current = controller.compute(x_est, xref(track_idx:end, :), scan);

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

    %% EKF Prediction
    % Simulate odometry with noise
    state_delta = x_true - x_prev;
    actual_delta_d = norm(state_delta(1:2));
    actual_delta_theta = state_delta(3);

    noisy_delta_d = actual_delta_d + process_noise_d * sqrt(dt) * randn;
    noisy_delta_theta = actual_delta_theta + process_noise_beta * sqrt(dt) * randn;

    ekf.predict(noisy_delta_d, noisy_delta_theta);

    %% EKF Update with LiDAR
    scan = scanner.scan(x_true, map_lines);
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

tracking_error = sqrt(sum((true_trajectory(1:2, :) - xref(:, 1:2)').^2, 1));

%% Visualization with Animation
fig_animation = figure('Name', 'NMPC+CBF with Line-based EKF', 'Position', [50 50 1800 1000]);

%% 1. Trajectory with map walls
subplot(2,3,1);
hold on; grid on; axis equal;

% Draw map walls once - extend to plot boundaries
plot_xlim = [0 30];
plot_ylim = [0 60];
line_extend = max([diff(plot_xlim), diff(plot_ylim)]);  % Use max dimension

for w = 1:num_walls
    alpha = map_lines(w, 1);
    d = map_lines(w, 2);
    n = [cos(alpha), sin(alpha)];
    p1 = d * n + line_extend * [-n(2), n(1)];
    p2 = d * n - line_extend * [-n(2), n(1)];
    plot([p1(1) p2(1)], [p1(2) p2(2)], ':', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);
end

% Initialize trajectory plots
h_ref_traj = plot(xref(:,1), xref(:,2), 'g--', 'LineWidth', 2, 'DisplayName', 'Reference');
h_true_traj = plot(true_trajectory(1, 1), true_trajectory(2, 1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'True');
h_est_traj = plot(estimated_trajectory(1, 1), estimated_trajectory(2, 1), 'r--', 'LineWidth', 1.5, 'DisplayName', 'EKF Est.');
h_true_robot = plot(true_trajectory(1, 1), true_trajectory(2, 1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'HandleVisibility', 'off');
h_est_robot = plot(estimated_trajectory(1, 1), estimated_trajectory(2, 1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
h_start = plot(true_trajectory(1,1), true_trajectory(2,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');

% LiDAR visualization
h_scan_hits = plot(NaN, NaN, 'r.', 'MarkerSize', 4, 'HandleVisibility', 'off');
h_scan_misses = plot(NaN, NaN, '.', 'Color', [0.5 0 0.5], 'MarkerSize', 4, 'HandleVisibility', 'off');
h_rays_hits = plot(NaN, NaN, 'Color', [1 0.5 0.5], 'LineWidth', 0.5, 'DisplayName', 'LiDAR Hits');
h_rays_misses = plot(NaN, NaN, 'Color', [0.7 0.5 0.7], 'LineWidth', 0.5, 'DisplayName', 'LiDAR Misses');
h_observed_lines = [];

xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
title('Trajectory Tracking with Line-based EKF', 'Interpreter', 'latex');
% legend('Interpreter', 'latex', 'Location', 'best');
xlim([0 30]); ylim([0 60]);

%% 2. Position Estimation Error
subplot(2,3,2);
hold on; grid on;
h_pos_error = plot(t(1), position_error(1), 'r-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Position Error [m]', 'Interpreter', 'latex');
title('Position Estimation Error', 'Interpreter', 'latex');
xlim([0 Tsim]);
ylim([0 max(position_error) * 1.1]);

%% 3. Heading Estimation Error
subplot(2,3,3);
hold on; grid on;
h_heading_error = plot(t(1), rad2deg(heading_error(1)), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Heading Error [deg]', 'Interpreter', 'latex');
title('Heading Estimation Error', 'Interpreter', 'latex');
xlim([0 Tsim]);
ylim([min(rad2deg(heading_error))-5, max(rad2deg(heading_error))+5]);

%% 4. Control inputs
subplot(2,3,4);
hold on; grid on;
h_v = plot(t(1), control_history(1, 1), 'b-', 'LineWidth', 1.5);
h_omega = plot(t(1), control_history(2, 1), 'r-', 'LineWidth', 1.5);
yline(v_max, 'b--', 'LineWidth', 1);
yline(omega_max, 'r--', 'LineWidth', 1);
yline(omega_min, 'r--', 'LineWidth', 1);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Control Input', 'Interpreter', 'latex');
title('Control Inputs', 'Interpreter', 'latex');
legend('$v$ [m/s]', '$\omega$ [rad/s]', 'Interpreter', 'latex');
xlim([0 Tsim]);
ylim([min([omega_min, min(control_history(2,:))])-0.5, max([v_max, max(control_history(1,:))])+0.5]);

%% 5. Tracking Error (True vs Reference)
subplot(2,3,5);
hold on; grid on;
h_tracking_error = plot(t(1), tracking_error(1), 'k-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Tracking Error [m]', 'Interpreter', 'latex');
title('Reference Tracking Error', 'Interpreter', 'latex');
xlim([0 Tsim]);
ylim([0 max(tracking_error) * 1.1]);

%% 6. EKF Uncertainty
subplot(2,3,6);
hold on; grid on;
h_cov_x = plot(t(1), covariance_history(1, 1), 'r-', 'LineWidth', 1.5);
h_cov_y = plot(t(1), covariance_history(2, 1), 'g-', 'LineWidth', 1.5);
h_cov_theta = plot(t(1), rad2deg(covariance_history(3, 1)), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Standard Deviation', 'Interpreter', 'latex');
title('EKF Uncertainty ($\sigma$)', 'Interpreter', 'latex');
legend('$\sigma_x$ [m]', '$\sigma_y$ [m]', '$\sigma_\theta$ [deg]', ...
       'Interpreter', 'latex', 'Location', 'best', 'AutoUpdate', 'off');
xlim([0 Tsim]);
ylim([0 max([covariance_history(1,:), covariance_history(2,:), rad2deg(covariance_history(3,:))]) * 1.1]);

sgtitle('NMPC+CBF with EKF Line-based State Estimation', 'Interpreter', 'latex');

%% Animation Loop
animation_step = 10;  % Update every N steps for smoother animation
for k = 1:animation_step:num_steps
    subplot(2,3,1);

    % Update trajectories
    set(h_true_traj, 'XData', true_trajectory(1, 1:k), 'YData', true_trajectory(2, 1:k));
    set(h_est_traj, 'XData', estimated_trajectory(1, 1:k), 'YData', estimated_trajectory(2, 1:k));

    % Update robot positions
    set(h_true_robot, 'XData', true_trajectory(1, k), 'YData', true_trajectory(2, k));
    set(h_est_robot, 'XData', estimated_trajectory(1, k), 'YData', estimated_trajectory(2, k));

    % Get current scan and robot state
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

    % Draw observed lines from RANSAC
    lines_observed = ransac_lines(scan, 0.015, 3);

    if ~isempty(h_observed_lines)
        delete(h_observed_lines(isvalid(h_observed_lines)));
    end
    h_observed_lines = [];

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

    % Update error plots
    subplot(2,3,2);
    set(h_pos_error, 'XData', t(1:k), 'YData', position_error(1:k));

    subplot(2,3,3);
    set(h_heading_error, 'XData', t(1:k), 'YData', rad2deg(heading_error(1:k)));

    % Update control plots
    subplot(2,3,4);
    set(h_v, 'XData', t(1:k), 'YData', control_history(1, 1:k));
    set(h_omega, 'XData', t(1:k), 'YData', control_history(2, 1:k));

    % Update tracking error
    subplot(2,3,5);
    set(h_tracking_error, 'XData', t(1:k), 'YData', tracking_error(1:k));

    % Update covariance plots
    subplot(2,3,6);
    set(h_cov_x, 'XData', t(1:k), 'YData', covariance_history(1, 1:k));
    set(h_cov_y, 'XData', t(1:k), 'YData', covariance_history(2, 1:k));
    set(h_cov_theta, 'XData', t(1:k), 'YData', rad2deg(covariance_history(3, 1:k)));

    drawnow('limitrate');
    pause(0.001);
end

figure(fig_animation);
fprintf('\nAnimation complete!\n');

%% Statistics
fprintf('\n========== Results ==========\n');
fprintf('Position Estimation Error:\n');
fprintf('  Final:   %.3f m\n', position_error(end));
fprintf('  Mean:    %.3f m\n', mean(position_error));
fprintf('  Max:     %.3f m\n', max(position_error));
fprintf('Heading Estimation Error:\n');
fprintf('  Final:   %.3f deg\n', rad2deg(abs(heading_error(end))));
fprintf('  Mean:    %.3f deg\n', rad2deg(mean(abs(heading_error))));
fprintf('Reference Tracking Error:\n');
fprintf('  Final:   %.3f m\n', tracking_error(end));
fprintf('  Mean:    %.3f m\n', mean(tracking_error));
fprintf('  Max:     %.3f m\n', max(tracking_error));
fprintf('\nFinal EKF Std Dev:\n');
fprintf('  Position (x):    %.4f m\n', covariance_history(1, end));
fprintf('  Position (y):    %.4f m\n', covariance_history(2, end));
fprintf('  Heading (theta): %.4f deg\n', rad2deg(covariance_history(3, end)));
fprintf('============================\n');
