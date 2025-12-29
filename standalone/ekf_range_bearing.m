clear;
close all;
clc;

%% Vehicle and Simulation Parameters
b = 0.5;                    % Track width (wheel separation) [m]
dt = 0.02;                  % Simulation/EKF time step [s] - 50 Hz
control_dt = 0.05;          % Control time step [s] - 20 Hz
simulation_time = 100;      % Total simulation time [s]
num_steps = simulation_time / dt;  % Total simulation steps
control_rate = control_dt / dt;    % Control updates every N steps

%% Define Trajectory
trajectory_type = 'curve';  % Options: 'linear', 'circular', 'curve'

% Create time vector for trajectory generation
t = (0:num_steps-1) * dt;

switch trajectory_type
    case 'linear'
        % Linear trajectory
        velocity_profile = 2.0 * ones(1, num_steps);        % Constant 2 m/s
        angular_velocity_profile = 0.0 * ones(1, num_steps); % Zero rotation
    case 'circular'
        % Circular/Arc trajectory
        velocity_profile = 1.0 * ones(1, num_steps); % Constant 1 m/s
        radius = 5; % 5m radius
        angular_velocity_profile = (velocity_profile / radius); % Constant angular velocity
    case 'curve'
        % Curved path with varying angular velocity
        velocity_profile = 1.5 * ones(1, num_steps); % Constant 1.5 m/s
        angular_velocity_profile = 0.2 * sin(2*pi*t/simulation_time); % Sinusoidal rotation [rad/s]
end


%% Process Noise

% For EKF (in odometry space: Δd, Δβ)
process_noise_d = 9.5003e-05;      % Distance increment noise [m]
process_noise_beta = 3.9080e-05;   % Heading increment noise [rad]
Q = diag([process_noise_d^2, process_noise_beta^2]);

% Pre-compute standard deviations for efficiency
Q_std = sqrt(diag(Q));

%% Measurement Noise

% Range and bearing measurements to beacons
measurement_noise_range = 0.018085189925279;   % Range measurement noise [m]
measurement_noise_bearing = 0.023174091647608;  % Bearing measurement noise [rad]
max_detection_range = 18.0;       % Maximum sensor range [m]

% R matrix: [r1, phi1, r2, phi2, r3, phi3] - 3 beacons, 2 measurements each
R = diag([measurement_noise_range^2, measurement_noise_bearing^2]);

% Pre-compute standard deviations for efficiency
R_std = sqrt(diag(R));

%% Beacon Positions
beacons = [
    3  25 1;   % Beacon 1
    1  1 2;   % Beacon 2
   11  3 3;   % Beacon 3
    30  0 4;   % Beacon 4
    15  6 5;   % Beacon 5
    20  5 6;   % Beacon 6
   10  7 7;   % Beacon 7
   30 34 8;   % Beacon 8
];
num_beacons = size(beacons, 1);
num_measurements = 2 * num_beacons;  % Range + bearing per beacon

%% Initial Conditions

% Initial state: start at origin, heading 45 degrees
true_state = [0; 0; pi/4];  % [x, y, theta]

% Initial state prediction (with error for EKF)
initial_state = true_state + [0.5; 0.5; 0.1];  % Just deviate a little

% Initial state covariance: with uncertainty for the state vector
initial_P = diag([0.5, 0.5, 0.1]);

% Create EKF instance
ekf = EKFLandmarks(initial_state, initial_P, beacons, Q, R);

%% Variables to show results
true_trajectory = zeros(3, num_steps);
estimated_trajectory = zeros(3, num_steps);
variance_history = zeros(3, num_steps);
control_history = zeros(2, num_steps);

%% Main Simulation Loop
v_current = velocity_profile(1);      % Initialize control
omega_current = angular_velocity_profile(1);

for step = 1:num_steps
    % Store previous state to compute actual displacement
    prev_state = true_state;

    % Update control at 20 Hz (every control_rate steps)
    if mod(step-1, round(control_rate)) == 0
        v_current = velocity_profile(step);
        omega_current = angular_velocity_profile(step);
    end

    % Update true state using ode45 with unicycle dynamics
    odefun = @(t, x) unicycle(x, [v_current; omega_current]);
    [~, x_traj] = ode45(odefun, [0, dt], true_state);
    true_state = x_traj(end, :)';

    %% Compute Odometry Measurements [Δd, Δβ] from true motion - Vectorized
    % This is what the EKF actually observes (with noise)
    state_delta = true_state - prev_state;
    actual_control = [norm(state_delta(1:2)); state_delta(3)];

    % Add odometry measurement noise (vectorized)
    noisy_control = actual_control + Q_std .* randn(2, 1);
    control_history(:, step) = noisy_control;

    true_trajectory(:, step) = true_state;

    %% Generate Measurements (Range and Bearing to each beacon) - Vectorized
    % Compute all ranges and bearings at once
    dx_beacons = beacons(:,1) - true_state(1);
    dy_beacons = beacons(:,2) - true_state(2);

    % Range measurements
    true_ranges = sqrt(dx_beacons.^2 + dy_beacons.^2);

    % Bearing measurements (relative to robot heading)
    true_bearings = atan2(dy_beacons, dx_beacons);
    relative_bearings = true_bearings - true_state(3);

    % Filter beacons within detection range
    in_range = true_ranges <= max_detection_range;
    visible_ranges = true_ranges(in_range);
    visible_bearings = relative_bearings(in_range);

    % Range and bearing measurements: [r1, phi1; r2, phi2; ...] for visible beacons only
    % Structure as Mx2 matrix (M visible beacons, 2 measurements each)
    if any(in_range)
        measurements = [visible_ranges, visible_bearings];

        % Add measurement noise (using pre-computed R_std)
        measurements = measurements + randn(sum(in_range), 2) .* R_std';

        % Normalize bearing angles to [-pi, pi]
        measurements(:, 2) = atan2(sin(measurements(:, 2)), cos(measurements(:, 2)));
    else
        % No beacons visible
        measurements = [];
    end

    %% EKF PREDICTION STEP
    % Control input: u = [Δd, Δβ]
    delta_d_hat = noisy_control(1);
    delta_beta_hat = noisy_control(2);

    % Use EKFLandmarks predict method
    ekf.predict(delta_d_hat, delta_beta_hat);

    %% EKF UPDATE STEP
    % Debug output for first few steps
    if step <= 5 || mod(step, 20) == 0
        fprintf('\n--- Step %d ---\n', step);
        fprintf('True position: [%.2f, %.2f, %.1f deg]\n', ...
                true_state(1), true_state(2), rad2deg(true_state(3)));
        fprintf('Est. position (before update): [%.2f, %.2f, %.1f deg]\n', ...
                ekf.x(1), ekf.x(2), rad2deg(ekf.x(3)));
        fprintf('Beacons in range: %d/%d\n', sum(in_range), num_beacons);
    end

    if ~isempty(measurements)
        % Get IDs of visible beacons
        visible_beacon_ids = find(in_range);
        % Update with known landmark correspondences
        ekf.update(measurements, visible_beacon_ids);
    end

    if step <= 5 || mod(step, 20) == 0
        fprintf('Est. position (after update):  [%.2f, %.2f, %.1f deg]\n', ...
                ekf.x(1), ekf.x(2), rad2deg(ekf.x(3)));
    end

    %% Store Results
    estimated_trajectory(:, step) = ekf.x;
    variance_history(:, step) = diag(ekf.P);
end

%% Compute Estimation Errors
position_error = sqrt(sum((true_trajectory(1:2,:) - estimated_trajectory(1:2,:)).^2, 1));
angle_diff = true_trajectory(3,:) - estimated_trajectory(3,:);
angle_error = abs(atan2(sin(angle_diff), cos(angle_diff)));

% Wrap angles to [-pi, pi] for visualization
true_trajectory_wrapped = true_trajectory;
estimated_trajectory_wrapped = estimated_trajectory;
true_trajectory_wrapped(3,:) = atan2(sin(true_trajectory(3,:)), cos(true_trajectory(3,:)));
estimated_trajectory_wrapped(3,:) = atan2(sin(estimated_trajectory(3,:)), cos(estimated_trajectory(3,:)));

%% Visualization with Animation
fig_animation = figure('Name', 'EKF with Range+Bearing Measurements', 'Position', [50 50 1400 900]);

% Plot 1: 2D Trajectory with Beacons
subplot(2,3,1);
hold on; grid on; axis equal;

% Draw beacons
plot(beacons(:,1), beacons(:,2), 'ks', 'MarkerSize', 5, 'MarkerFaceColor', 'k', 'DisplayName', sprintf('Beacons (%d)', num_beacons));
% Label each beacon
for i = 1:num_beacons
    text(beacons(i,1)+0.2, beacons(i,2)+0.2, sprintf('%d', i), 'FontSize', 10, 'Color', 'k');
end

h_true_traj = plot(true_trajectory(1, 1), true_trajectory(2, 1), 'b-', 'LineWidth', 2);
h_est_traj = plot(estimated_trajectory(1, 1), estimated_trajectory(2, 1), 'r--', 'LineWidth', 2);
h_true_robot = plot(true_trajectory(1, 1), true_trajectory(2, 1), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
h_est_robot = plot(estimated_trajectory(1, 1), estimated_trajectory(2, 1), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');

% Detection range circle that follows the robot
theta_range = linspace(0, 2*pi, 100);
x_range = true_trajectory(1, 1) + max_detection_range * cos(theta_range);
y_range = true_trajectory(2, 1) + max_detection_range * sin(theta_range);
h_range_circle = plot(x_range, y_range, 'b--', 'LineWidth', 1.5, 'HandleVisibility', 'off');

h_measurements = plot(NaN, NaN, 'g-', 'LineWidth', 1);  % Lines from robot to visible beacons

xlabel('X [m]'); ylabel('Y [m]');
title(sprintf('Trajectory (%s)', trajectory_type));
xlim([min(true_trajectory(1,:))-5, max(true_trajectory(1,:))+5]);
ylim([min(true_trajectory(2,:))-5, max(true_trajectory(2,:))+5]);

% Plot 2: Position Error
time_vector = (0:num_steps-1) * dt;
subplot(2,3,2);
hold on; grid on;
h_error = plot(time_vector(1), position_error(1), 'm-', 'LineWidth', 2);
h_angle_error = plot(time_vector(1), angle_error(1), 'g-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('State Error');
title('State Estimation Error');
xlim([0 simulation_time]);
ylim([0 max([position_error angle_error]) * 1.1]);

% Plot 3: Covariance
subplot(2,3,3);
hold on; grid on;
h_cov_x = plot(time_vector(1), sqrt(variance_history(1, 1)), 'r-', 'LineWidth', 1.5);
h_cov_y = plot(time_vector(1), sqrt(variance_history(2, 1)), 'g-', 'LineWidth', 1.5);
h_cov_theta = plot(time_vector(1), sqrt(variance_history(3, 1)), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Standard Deviation');
title('State Uncertainty');
legend('$\sigma_x$ [m]', '$\sigma_y$ [m]', '$\sigma_\theta$ [rad]', 'Location', 'best', 'AutoUpdate', 'off');
xlim([0 simulation_time]);
ylim([0 max([sqrt(variance_history(1,:)), sqrt(variance_history(2,:)), sqrt(variance_history(3,:))]) * 1.1]);

% Plot 4: X Position
subplot(2,3,4);
hold on; grid on;
h_x_true = plot(time_vector(1), true_trajectory(1, 1), 'b-', 'LineWidth', 1.5);
h_x_est = plot(time_vector(1), estimated_trajectory(1, 1), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('X [m]');
title('X Position');
legend('True', 'Estimated', 'Location', 'best', 'AutoUpdate', 'off');
xlim([0 simulation_time]);
ylim([min([true_trajectory(1,:), estimated_trajectory(1,:)])-1, max([true_trajectory(1,:), estimated_trajectory(1,:)])+1]);
grid on;

% Plot 5: Y Position
subplot(2,3,5);
hold on; grid on;
h_y_true = plot(time_vector(1), true_trajectory(2, 1), 'b-', 'LineWidth', 1.5);
h_y_est = plot(time_vector(1), estimated_trajectory(2, 1), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Y [m]');
title('Y Position');
legend('True', 'Estimated', 'Location', 'best', 'AutoUpdate', 'off');
xlim([0 simulation_time]);
ylim([min([true_trajectory(2,:), estimated_trajectory(2,:)])-1, max([true_trajectory(2,:), estimated_trajectory(2,:)])+1]);
grid on;

% Plot 6: Heading Angle
subplot(2,3,6);
hold on; grid on;
h_theta_true = plot(time_vector(1), rad2deg(true_trajectory_wrapped(3, 1)), 'b-', 'LineWidth', 1.5);
h_theta_est = plot(time_vector(1), rad2deg(estimated_trajectory_wrapped(3, 1)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Heading [deg]');
title('Heading Angle');
legend('True', 'Estimated', 'Location', 'best', 'AutoUpdate', 'off');
xlim([0 simulation_time]);
ylim([-190, 190]);  % Fixed range for wrapped angles
grid on;

%% Animation Loop
animation_step = 2;  % Update every N steps
for k = 1:animation_step:num_steps
    subplot(2,3,1);

    % Update trajectories
    set(h_true_traj, 'XData', true_trajectory(1, 1:k), 'YData', true_trajectory(2, 1:k));
    set(h_est_traj, 'XData', estimated_trajectory(1, 1:k), 'YData', estimated_trajectory(2, 1:k));

    % Update robot positions
    set(h_true_robot, 'XData', true_trajectory(1, k), 'YData', true_trajectory(2, k));
    set(h_est_robot, 'XData', estimated_trajectory(1, k), 'YData', estimated_trajectory(2, k));

    % Update detection range circle to follow the robot
    x_range = true_trajectory(1, k) + max_detection_range * cos(theta_range);
    y_range = true_trajectory(2, k) + max_detection_range * sin(theta_range);
    set(h_range_circle, 'XData', x_range, 'YData', y_range);

    % Draw lines to visible beacons
    robot_pos = true_trajectory(1:2, k);
    dx_beacons = beacons(:,1) - robot_pos(1);
    dy_beacons = beacons(:,2) - robot_pos(2);
    ranges = sqrt(dx_beacons.^2 + dy_beacons.^2);
    visible = ranges <= max_detection_range;

    if any(visible)
        visible_beacons = beacons(visible, :);
        n_visible = sum(visible);
        line_x = zeros(3 * n_visible, 1);
        line_y = zeros(3 * n_visible, 1);
        for i = 1:n_visible
            idx = 3 * (i - 1);
            line_x(idx + 1) = robot_pos(1);
            line_y(idx + 1) = robot_pos(2);
            line_x(idx + 2) = visible_beacons(i, 1);
            line_y(idx + 2) = visible_beacons(i, 2);
            line_x(idx + 3) = NaN;
            line_y(idx + 3) = NaN;
        end
        set(h_measurements, 'XData', line_x, 'YData', line_y);
    else
        set(h_measurements, 'XData', NaN, 'YData', NaN);
    end

    % Update error plot
    subplot(2,3,2);
    set(h_error, 'XData', time_vector(1:k), 'YData', position_error(1:k));
    set(h_angle_error, 'XData', time_vector(1:k), 'YData', angle_error(1:k));

    % Update covariance plot
    subplot(2,3,3);
    set(h_cov_x, 'XData', time_vector(1:k), 'YData', sqrt(variance_history(1, 1:k)));
    set(h_cov_y, 'XData', time_vector(1:k), 'YData', sqrt(variance_history(2, 1:k)));
    set(h_cov_theta, 'XData', time_vector(1:k), 'YData', sqrt(variance_history(3, 1:k)));

    % Update X position plot
    subplot(2,3,4);
    set(h_x_true, 'XData', time_vector(1:k), 'YData', true_trajectory(1, 1:k));
    set(h_x_est, 'XData', time_vector(1:k), 'YData', estimated_trajectory(1, 1:k));

    % Update Y position plot
    subplot(2,3,5);
    set(h_y_true, 'XData', time_vector(1:k), 'YData', true_trajectory(2, 1:k));
    set(h_y_est, 'XData', time_vector(1:k), 'YData', estimated_trajectory(2, 1:k));

    % Update heading plot
    subplot(2,3,6);
    set(h_theta_true, 'XData', time_vector(1:k), 'YData', rad2deg(true_trajectory_wrapped(3, 1:k)));
    set(h_theta_est, 'XData', time_vector(1:k), 'YData', rad2deg(estimated_trajectory_wrapped(3, 1:k)));

    drawnow('limitrate');
    pause(0.001);
end

figure(fig_animation);
fprintf('Animation complete!\n');

%% Display Statistics
fprintf('\n========== EKF with range+bearing measurements - Results ==========\n');
fprintf('True Dynamics:          Differential Drive [v, ω]\n');
fprintf('EKF Control Input:      Odometry [Δd, Δβ]\n');
fprintf('Measurement Model:      Range + Bearing to %d beacons\n', num_beacons);
fprintf('Trajectory Type:        %s\n', trajectory_type);
fprintf('Track Width (b):        %.2f m\n', b);
fprintf('EKF Time Step:          %.3f s (50 Hz)\n', dt);
fprintf('Control Time Step:      %.3f s (20 Hz)\n', control_dt);
fprintf('Simulation Time:        %.2f s\n', simulation_time);
fprintf('Number of Steps:        %d\n', num_steps);
fprintf('\nMeasurement Noise:\n');
fprintf('  Range Std Dev:        %.3f m\n', measurement_noise_range);
fprintf('  Bearing Std Dev:      %.3f rad (%.2f deg)\n', measurement_noise_bearing, rad2deg(measurement_noise_bearing));
fprintf('  Max Detection Range:  %.1f m\n', max_detection_range);
fprintf('\nFinal Position Error:   %.3f m\n', position_error(end));
fprintf('Mean Position Error:    %.3f m\n', mean(position_error));
fprintf('Max Position Error:     %.3f m\n', max(position_error));
fprintf('Final Angle Error:      %.3f deg\n', rad2deg(angle_error(end)));
fprintf('\nFinal Std Dev (x):      %.4f m\n', sqrt(variance_history(1,end)));
fprintf('Final Std Dev (y):      %.4f m\n', sqrt(variance_history(2,end)));
fprintf('Final Std Dev (theta):  %.4f rad\n', sqrt(variance_history(3,end)));
fprintf('=================================================================\n\n');
