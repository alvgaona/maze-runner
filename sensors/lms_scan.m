function scan = lms_scan(robot_pose, map_lines, max_range, noise_std, model)
    % Simulates LMS200 or LMS100 laser scanner
    % Returns N×2 matrix [range(m), bearing(rad)]
    %
    % Inputs:
    %   robot_pose   - [x, y, theta] current pose
    %   map_lines    - N×2 matrix of lines in Hesse form [alpha, d]
    %   max_range    - maximum sensor range (m)
    %   noise_std    - range measurement noise std dev (m)
    %   model - 'LMS200' (default) or 'LMS100'

    arguments
        robot_pose (3,1) double
        map_lines (:,2) double
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

    % For each wall, compute intersection with laser beams
    for w = 1:size(map_lines, 1)
        alpha = map_lines(w, 1);
        d_wall = map_lines(w, 2);
        n = [cos(alpha); sin(alpha)];

        % Check each laser beam for intersection
        for k = 1:numel(angles)
            phi = angles(k) + robot_pose(3);
            v = [cos(phi); sin(phi)];
            denom = n' * v;
            if abs(denom) > 1e-6
                t = (d_wall - n' * robot_pose(1:2)) / denom;
                if t > 0 && t < max_range
                    ranges(k) = min(ranges(k), t);
                end
            end
        end
    end

    % Mark out-of-range readings as NaN
    ranges(isinf(ranges)) = NaN;

    % Add measurement noise
    ranges = ranges + noise_std * randn(size(ranges));

    % Return [range, bearing] pairs
    scan = [ranges(:), angles(:)];
end
