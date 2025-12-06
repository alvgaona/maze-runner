clc; clear; close all;

%% Test new generic LiDAR scan with square obstacle

corridor_width = 4;
corridor_length = 12;

% Define obstacles
obstacles = {};
obstacles{end+1} = struct('type', 'wall', 'p1', [corridor_width/2; -1], 'p2', [corridor_width/2; corridor_length]);
obstacles{end+1} = struct('type', 'wall', 'p1', [-corridor_width/2; -1], 'p2', [-corridor_width/2; corridor_length]);

% Square obstacle
obstacle_center = [0; 5];
obstacle_size = 1.2;
obstacles{end+1} = struct('type', 'square', 'center', obstacle_center, 'size', obstacle_size);

% Robot position
robot_pose = [0; 1; pi/2];  % Near bottom, pointing up

% Get scan
scan = lms_scan_new(robot_pose, obstacles, 10.0, 0.0, 'LMS200');

% Visualize
figure('Position', [100 100 1400 600]);

% Left: Map view
subplot(1,2,1);
hold on; grid on; axis equal;

% Draw corridor
plot([corridor_width/2, corridor_width/2], [-1, corridor_length], 'k-', 'LineWidth', 2);
plot([-corridor_width/2, -corridor_width/2], [-1, corridor_length], 'k-', 'LineWidth', 2);

% Draw square
half_size = obstacle_size / 2;
x_square = obstacle_center(1) + [-half_size, half_size, half_size, -half_size, -half_size];
y_square = obstacle_center(2) + [-half_size, -half_size, half_size, half_size, -half_size];
plot(x_square, y_square, 'r-', 'LineWidth', 2);
fill(x_square, y_square, [1 0.8 0.8], 'FaceAlpha', 0.3);

% Draw robot
plot(robot_pose(1), robot_pose(2), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
quiver(robot_pose(1), robot_pose(2), 0.5*cos(robot_pose(3)), 0.5*sin(robot_pose(3)), ...
       0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Draw LiDAR rays
valid = ~isnan(scan(:,1));
ranges = scan(valid, 1);
bearings = scan(valid, 2);

x_points = robot_pose(1) + ranges .* cos(bearings + robot_pose(3));
y_points = robot_pose(2) + ranges .* sin(bearings + robot_pose(3));

% Draw ALL rays to see what's happening
for i = 1:length(ranges)
    plot([robot_pose(1), x_points(i)], [robot_pose(2), y_points(i)], 'c-', 'LineWidth', 0.5);
end
plot(x_points, y_points, 'r.', 'MarkerSize', 8);

% Also draw rays that didn't hit anything (for debugging)
invalid = isnan(scan(:,1));
invalid_bearings = scan(invalid, 2);
% Draw these rays at max range to see where they're pointing
for i = 1:length(invalid_bearings)
    phi = invalid_bearings(i) + robot_pose(3);
    x_end = robot_pose(1) + 10 * cos(phi);
    y_end = robot_pose(2) + 10 * sin(phi);
    plot([robot_pose(1), x_end], [robot_pose(2), y_end], 'g-', 'LineWidth', 0.2, 'Color', [0.5 0.5 0.5 0.3]);
end

xlabel('x [m]');
ylabel('y [m]');
title('LiDAR Scan with Square Obstacle');
xlim([-3, 3]);
ylim([-1, 10]);
legend('Wall', '', 'Obstacle', '', 'Robot', '', 'Scan points', 'Location', 'best');

% Right: Range plot
subplot(1,2,2);
plot(rad2deg(scan(:,2)), scan(:,1), 'b.', 'MarkerSize', 4);
grid on;
xlabel('Bearing [deg]');
ylabel('Range [m]');
title('LiDAR Range Measurements');
ylim([0, 10]);

% Statistics
fprintf('LiDAR scan statistics:\n');
fprintf('Valid measurements: %d / %d\n', sum(valid), length(scan(:,1)));
fprintf('Min range: %.2f m\n', min(ranges));
fprintf('Max range: %.2f m\n', max(ranges));
fprintf('Expected distance to obstacle front edge: %.2f m\n', ...
        obstacle_center(2) - half_size - robot_pose(2));
fprintf('Expected distance to corridor walls: %.2f m\n', corridor_width/2);
