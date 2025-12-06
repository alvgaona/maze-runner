function scan = lms_scan_new(robot_pose, obstacles, max_range, noise_std, model)
    % Generic LMS laser scanner using raycast intersection with obstacles
    % Returns N×2 matrix [range(m), bearing(rad)]
    %
    % Inputs:
    %   robot_pose   - [x, y, theta] current pose
    %   obstacles    - Cell array of obstacles, each as struct with:
    %                  .type = 'wall' | 'square' | 'circle'
    %                  For 'wall': .p1 = [x1; y1], .p2 = [x2; y2] (endpoints)
    %                  For 'square': .center = [x; y], .size = side_length
    %                  For 'circle': .center = [x; y], .radius = r
    %   max_range    - maximum sensor range (m)
    %   noise_std    - range measurement noise std dev (m)
    %   model        - 'LMS200' (default) or 'LMS100'

    arguments
        robot_pose (3,1) double
        obstacles cell
        max_range (1,1) double {mustBePositive}
        noise_std (1,1) double {mustBeNonnegative}
        model {mustBeMember(model, {'LMS200', 'LMS100'})} = 'LMS200'
    end

    % Configure scanner parameters
    switch model
        case 'LMS200'
            % LMS200: 180-degree field of view, 181 beams (1-degree resolution)
            angles = linspace(-pi/2, pi/2, 181);
        case 'LMS100'
            % LMS100: 270-degree field of view, 541 beams (0.5-degree resolution)
            angles = linspace(-3*pi/4, 3*pi/4, 541);
    end

    ranges = inf(size(angles));

    % Robot position
    rx = robot_pose(1);
    ry = robot_pose(2);
    rtheta = robot_pose(3);

    %% Raycast for each beam
    for k = 1:numel(angles)
        % Ray direction in world frame
        phi = angles(k) + rtheta;
        ray_dir = [cos(phi); sin(phi)];

        % Check intersection with all obstacles
        for obs_idx = 1:length(obstacles)
            obs = obstacles{obs_idx};

            switch obs.type
                case 'wall'
                    % Wall defined by two endpoints
                    t = raycast_segment(rx, ry, ray_dir, obs.p1, obs.p2);
                    if t > 0 && t < max_range
                        ranges(k) = min(ranges(k), t);
                    end

                case 'square'
                    % Square defined by center and size
                    center = obs.center;
                    half_size = obs.size / 2;

                    % Four edges of square
                    corners = [
                        center(1) - half_size, center(2) - half_size;
                        center(1) + half_size, center(2) - half_size;
                        center(1) + half_size, center(2) + half_size;
                        center(1) - half_size, center(2) + half_size;
                    ];

                    % Check each edge
                    for i = 1:4
                        p1 = corners(i, :)';
                        p2 = corners(mod(i, 4) + 1, :)';
                        t = raycast_segment(rx, ry, ray_dir, p1, p2);
                        if t > 0 && t < max_range
                            ranges(k) = min(ranges(k), t);
                        end
                    end

                case 'circle'
                    % Circle defined by center and radius
                    t = raycast_circle(rx, ry, ray_dir, obs.center, obs.radius);
                    if t > 0 && t < max_range
                        ranges(k) = min(ranges(k), t);
                    end
            end
        end
    end

    %% Finalize scan
    % Mark out-of-range readings as NaN
    ranges(isinf(ranges)) = NaN;

    % Add measurement noise
    ranges = ranges + noise_std * randn(size(ranges));

    % Return [range, bearing] pairs
    scan = [ranges(:), angles(:)];
end

%% Helper Functions

function t = raycast_segment(rx, ry, ray_dir, p1, p2)
    % Raycast intersection with line segment
    % Returns distance t along ray, or -1 if no intersection
    %
    % Ray: P = [rx, ry] + t * ray_dir
    % Segment: P = p1 + s * (p2 - p1), where s ∈ [0, 1]

    rdx = ray_dir(1);
    rdy = ray_dir(2);

    sdx = p2(1) - p1(1);
    sdy = p2(2) - p1(2);

    % Solve: [rx, ry] + t*[rdx, rdy] = p1 + s*[sdx, sdy]
    % Rearrange: t*[rdx, rdy] - s*[sdx, sdy] = p1 - [rx, ry]
    % In matrix form:
    % [rdx  -sdx] [t]   [p1(1) - rx]
    % [rdy  -sdy] [s] = [p1(2) - ry]

    denom = rdx * (-sdy) - rdy * (-sdx);

    if abs(denom) < 1e-6
        % Parallel or collinear
        t = -1;
        return;
    end

    % Cramer's rule
    dx = p1(1) - rx;
    dy = p1(2) - ry;

    t = (dx * (-sdy) - dy * (-sdx)) / denom;
    s = (rdx * dy - rdy * dx) / denom;

    % Check if intersection is on both ray and segment
    if s >= 0 && s <= 1 && t > 0
        % Valid intersection
        return;
    else
        t = -1;
    end
end

function t = raycast_circle(rx, ry, ray_dir, center, radius)
    % Raycast intersection with circle
    % Returns distance t to nearest intersection, or -1 if no hit
    %
    % Ray: P = [rx, ry] + t * ray_dir
    % Circle: ||P - center||^2 = radius^2

    % Vector from ray origin to circle center
    dx = center(1) - rx;
    dy = center(2) - ry;

    % Quadratic equation: ||[rx, ry] + t*ray_dir - center||^2 = radius^2
    % a*t^2 + b*t + c = 0
    a = ray_dir(1)^2 + ray_dir(2)^2;  % Should be 1 for normalized ray
    b = -2 * (dx * ray_dir(1) + dy * ray_dir(2));
    c = dx^2 + dy^2 - radius^2;

    discriminant = b^2 - 4*a*c;

    if discriminant < 0
        % No intersection
        t = -1;
        return;
    end

    % Two solutions (entry and exit points)
    sqrt_disc = sqrt(discriminant);
    t1 = (-b - sqrt_disc) / (2*a);
    t2 = (-b + sqrt_disc) / (2*a);

    % Return nearest positive intersection
    if t1 > 0
        t = t1;
    elseif t2 > 0
        t = t2;
    else
        t = -1;
    end
end
