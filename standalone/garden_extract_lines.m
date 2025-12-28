clear; clc; close all;

%% Load Occupancy Grid Map
load('map/gardenMap.mat');
occupancy_grid_map = garden(:,:,1);

%% Extract lines
resolution = 10;  % Pixels per unit (meter)
map_lines = extract_map_lines(occupancy_grid_map, resolution);

%% Visualization
figure('Name', 'Line Extraction - Hough Transform');

imshow(~occupancy_grid_map);
hold on;

[height, width] = size(occupancy_grid_map);

for i = 1:size(map_lines, 1)
    alpha = map_lines(i, 1);
    d = map_lines(i, 2) * resolution;  % Convert back to pixels
    
    % Normal vector
    n = [cos(alpha), sin(alpha)];
    
    % Point on line closest to origin
    p0 = d * n;
    
    % Direction along line (perpendicular to normal)
    v = [-n(2), n(1)];
    
    % Draw line across image
    line_length = max(width, height);
    p1 = p0 + line_length * v;
    p2 = p0 - line_length * v;
    
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'r-', 'LineWidth', 1.5);
end

title(sprintf('Lines in Hesse Form (%d)', size(map_lines, 1)));
hold off;

%% Display Results
fprintf('========== Extracted Lines (Hesse Form) ==========\n');
fprintf('  #   Alpha [rad]   Alpha [deg]   Distance [m]\n');
fprintf('---------------------------------------------------\n');
for i = 1:size(map_lines, 1)
    fprintf('%3d   %10.4f   %11.2f   %11.4f\n', ...
        i, map_lines(i, 1), rad2deg(map_lines(i, 1)), map_lines(i, 2));
end
fprintf('===================================================\n\n');

%% Save Results
save('map/garden_lines.mat', 'map_lines');
