classdef EKFLines < handle
    % EKFLines - Extended Kalman Filter for Line-based Localization

    properties
        x           % State estimate [x; y; theta] (3x1)
        P           % State covariance matrix (3x3)
        map_lines   % Map lines in Hesse form [alpha, d] (Nx2)
        Q           % Process noise covariance [Δd; Δβ] (2x2)
        chi2_thresh % Chi-squared threshold for outlier rejection
    end

    methods
        function obj = EKFLines(initial_state, initial_covariance, ...
                                map_lines, process_noise_cov, chi2_threshold)
            % Constructor
            obj.x = initial_state(:);
            obj.P = initial_covariance;
            obj.map_lines = map_lines;
            obj.Q = process_noise_cov;
            obj.chi2_thresh = chi2_threshold;
        end

        function predict(obj, delta_d, delta_theta)
            % predict - EKF prediction step using odometry
            theta_k = obj.x(3);

            % Midpoint odometry model
            theta_mid = theta_k + delta_theta / 2;
            obj.x = [
                obj.x(1) + delta_d * cos(theta_mid);
                obj.x(2) + delta_d * sin(theta_mid);
                theta_k + delta_theta
            ];

            % Normalize heading angle
            obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));

            % Jacobian of transition function with respect to state
            % f(x,y,θ) = [x + Δd*cos(θ + Δβ/2); y + Δd*sin(θ + Δβ/2); θ + Δβ]
            A = [
                1, 0, -delta_d * sin(theta_mid);
                0, 1,  delta_d * cos(theta_mid);
                0, 0,  1
            ];

            % Jacobian with respect to odometry input u = [Δd; Δβ]
            W = [
                cos(theta_mid),  -0.5 * delta_d * sin(theta_mid);
                sin(theta_mid),   0.5 * delta_d * cos(theta_mid);
                0,                1
            ];

            % Predict covariance
            obj.P = A * obj.P * A' + W * obj.Q * W';
        end

        function num_updates = update(obj, observed_lines)
            % update - EKF update step using observed line features
            num_updates = 0;

            for j = 1:size(observed_lines, 1)
                alpha_observed = observed_lines(j, 1);  % Angle [rad]
                d_observed = observed_lines(j, 2);      % Distance [m]
                sigma_alpha = observed_lines(j, 3);     % Angular uncertainty [rad]
                sigma_d = observed_lines(j, 4);         % Distance uncertainty [m]

                % Data association: Match observed line to map line
                [alpha_map, d_map, match_found] = obj.associateLine(alpha_observed, d_observed);

                if ~match_found
                    continue;  % Skip if no good match found
                end

                % Predicted measurement
                normal_map = [cos(alpha_map); sin(alpha_map)];
                theta_predicted = obj.x(3);
                alpha_predicted = alpha_map - theta_predicted;
                d_predicted = d_map - normal_map' * obj.x(1:2);

                % Innovation (measurement residual)
                innovation_angle = atan2(sin(alpha_observed - alpha_predicted), ...
                                       cos(alpha_observed - alpha_predicted));
                innovation_distance = d_observed - d_predicted;
                innovation = [innovation_angle; innovation_distance];

                % Measurement Jacobian H (2x3)
                % h(x,y,θ) = [alpha_map - θ; d_map - cos(alpha_map)*x - sin(alpha_map)*y]
                H = [0,              0,              -1;
                     -cos(alpha_map), -sin(alpha_map), 0];

                % Measurement noise covariance
                R = diag([sigma_alpha^2, sigma_d^2]);

                % Innovation covariance
                S = H * obj.P * H' + R;

                % Mahalanobis distance gating
                mahalanobis_dist = innovation' / S * innovation;
                if mahalanobis_dist > obj.chi2_thresh
                    continue;  % Reject outlier
                end

                % Kalman update
                K = obj.P * H' / S;
                obj.x = obj.x + K * innovation;
                obj.P = (eye(3) - K * H) * obj.P;

                % Normalize heading after update
                obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));

                num_updates = num_updates + 1;
            end
        end
    end

    methods (Access = private)
        function [alpha_map, d_map, match_found] = associateLine(obj, alpha_observed, d_observed)
            % associateLine - Data association for observed line
            %
            % Finds the closest matching map line based on both angle and distance.
            %
            % Args:
            %   alpha_observed: Observed line angle in robot frame [rad]
            %   d_observed: Observed line distance in robot frame [m]
            %
            % Returns:
            %   alpha_map: Matched map line angle in world frame [rad]
            %   d_map: Matched map line distance from origin [m]
            %   match_found: Boolean indicating if match was found

            % Transform observed angle to world frame
            alpha_world = alpha_observed + obj.x(3);

            % Find closest matching line by angle and distance
            num_walls = size(obj.map_lines, 1);
            angle_differences = zeros(num_walls, 1);
            distance_differences = zeros(num_walls, 1);

            for w = 1:num_walls
                % Angular difference
                angle_differences(w) = abs(atan2(...
                    sin(obj.map_lines(w, 1) - alpha_world), ...
                    cos(obj.map_lines(w, 1) - alpha_world)));

                % Predicted distance for this map line given current robot pose
                alpha_map_w = obj.map_lines(w, 1);
                d_map_w = obj.map_lines(w, 2);
                normal_map_w = [cos(alpha_map_w); sin(alpha_map_w)];
                d_predicted = d_map_w - normal_map_w' * obj.x(1:2);

                % Distance difference
                distance_differences(w) = abs(d_observed - d_predicted);
            end

            angle_weight = 2.0;
            dist_weight = 1.0;

            combined_score = angle_weight * angle_differences + dist_weight * distance_differences;

            [~, idx] = min(combined_score);
            min_angle_diff = angle_differences(idx);
            min_dist_diff = distance_differences(idx);

            match_found = (min_angle_diff < deg2rad(30)) && (min_dist_diff < 0.5);

            if match_found
                alpha_map = obj.map_lines(idx, 1);
                d_map = obj.map_lines(idx, 2);
            else
                alpha_map = NaN;
                d_map = NaN;
            end
        end
    end
end
