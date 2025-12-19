classdef NMPCCBFController < handle
    % NMPCCBFController - Nonlinear MPC with Control Barrier Functions

    properties (SetAccess = private)
        % NMPC parameters
        N               % Prediction horizon length
        dt              % Time step [s]
        Q               % State tracking weights (3x3)
        R               % Control effort weights (2x2)

        % Control constraints
        v_min           % Minimum linear velocity [m/s]
        v_max           % Maximum linear velocity [m/s]
        omega_min       % Minimum angular velocity [rad/s]
        omega_max       % Maximum angular velocity [rad/s]

        % CBF parameters
        d_safe          % Safety radius [m]
        alpha_cbf       % CBF aggressiveness parameter
        scan_downsample % Downsample factor for scan constraints
        constraint_range % Maximum range for CBF constraints [m]
        slack_penalty   % Penalty weight for constraint violations
        use_slack       % Enable slack variables for soft constraints

        % CasADi/IPOPT settings
        solver_options  % IPOPT options
        max_iterations  % Maximum solver iterations

        % Pre-built base optimization problem (no CBF)
        opti_base       % CasADi Opti instance (base problem)
        U_var           % Control decision variables (2xN)
        X_var           % State trajectory variables (3xN+1)
        x0_param        % Initial state parameter (3x1)
        xref_param      % Reference trajectory parameter (N+1 x 3)

        % State for warm-starting
        last_u          % Last control input (2xN)

        % Scan FoV
        scan_fov        % FoV of interest for the scans
    end

    methods
        function obj = NMPCCBFController(options)
            % NMPCCBFController Constructor
            arguments
                options.HorizonLength (1,1) double {mustBePositive} = 20
                options.TimeStep (1,1) double {mustBePositive} = 0.1
                options.StateWeights (1,3) double = [1, 1, 0.1]
                options.ControlWeights (1,2) double = [1, 1]
                options.VelocityLimits (1,2) double = [0, 1.5]
                options.AngularLimits (1,2) double = [-2, 2]
                options.SafetyRadius (1,1) double {mustBePositive} = 0.6
                options.AlphaCBF (1,1) double {mustBePositive} = 1.0
                options.ScanDownsample (1,1) double {mustBePositive, mustBeInteger} = 50
                options.ConstraintRange (1,1) double {mustBePositive} = 2.0
                options.MaxIterations (1,1) double {mustBePositive} = 100
                options.UseSlack (1,1) logical = false
                options.SlackPenalty (1,1) double {mustBePositive} = 1000
                options.FoV (1,1) double {mustBePositive} = 30
            end

            obj.N = options.HorizonLength;
            obj.dt = options.TimeStep;
            obj.Q = diag(options.StateWeights);
            obj.R = diag(options.ControlWeights);

            obj.v_min = options.VelocityLimits(1);
            obj.v_max = options.VelocityLimits(2);
            obj.omega_min = options.AngularLimits(1);
            obj.omega_max = options.AngularLimits(2);

            obj.d_safe = options.SafetyRadius;
            obj.alpha_cbf = options.AlphaCBF;
            obj.scan_downsample = options.ScanDownsample;
            obj.constraint_range = options.ConstraintRange;
            obj.use_slack = options.UseSlack;
            obj.slack_penalty = options.SlackPenalty;

            % Configure IPOPT solver options
            obj.solver_options = struct();
            obj.solver_options.ipopt.print_level = 0;
            obj.solver_options.ipopt.max_iter = options.MaxIterations;
            obj.solver_options.ipopt.acceptable_tol = 1e-6;
            obj.solver_options.ipopt.acceptable_obj_change_tol = 1e-6;
            obj.solver_options.ipopt.warm_start_init_point = 'yes';
            obj.solver_options.print_time = false;
            obj.solver_options.verbose = false;

            obj.last_u = zeros(2, obj.N);

            obj.scan_fov = deg2rad(options.FoV);  % Convert to radians

            % Build base optimization problem once (no CBF constraints)
            obj.buildBaseOptimizationProblem();
        end

        function buildBaseOptimizationProblem(obj)
            % Build the base NMPC problem structure (no CBF, those added per solve)
            import casadi.*

            obj.opti_base = casadi.Opti();

            % Decision variables
            obj.U_var = obj.opti_base.variable(2, obj.N);
            obj.X_var = obj.opti_base.variable(3, obj.N+1);

            % Parameters
            obj.x0_param = obj.opti_base.parameter(3, 1);
            obj.xref_param = obj.opti_base.parameter(obj.N+1, 3);

            % Initial condition constraint
            obj.opti_base.subject_to(obj.X_var(:,1) == obj.x0_param);

            % Dynamics constraints
            for k = 1:obj.N
                x_next = obj.X_var(:,k) + obj.dt * [
                    obj.U_var(1,k)*cos(obj.X_var(3,k));
                    obj.U_var(1,k)*sin(obj.X_var(3,k));
                    obj.U_var(2,k)
                ];
                obj.opti_base.subject_to(obj.X_var(:,k+1) == x_next);
            end

            % Control constraints
            obj.opti_base.subject_to(obj.v_min <= obj.U_var(1,:) <= obj.v_max);
            obj.opti_base.subject_to(obj.omega_min <= obj.U_var(2,:) <= obj.omega_max);

            % Note: Cost function will be set in compute() to allow for slack penalty
            % Set solver
            obj.opti_base.solver('ipopt', obj.solver_options);
        end

        function u = compute(obj, x, xref, scan)
            % compute - Compute control signal using pre-built Opti + dynamic CBF
            import casadi.*

            x = x(:);

            % Trajectory provided - use first N+1 points
            xref_horizon = xref(1:min(obj.N+1, size(xref,1)), :);

            % Pad if needed
            if size(xref_horizon, 1) < obj.N + 1
                xref_horizon = [xref_horizon; repmat(xref_horizon(end,:), ...
                    obj.N + 1 - size(xref_horizon, 1), 1)];
            end

            valid = ~isnan(scan(:,1)) & scan(:,1) < obj.constraint_range & ...
                    abs(scan(:,2)) <= obj.scan_fov;
            valid_idx = find(valid);

            if ~isempty(valid_idx)
                valid_idx = valid_idx(1:obj.scan_downsample:end);
                ranges = scan(valid_idx, 1);
                bearings = scan(valid_idx, 2);
                n_obstacles = length(ranges);
            else
                ranges = [];
                bearings = [];
                n_obstacles = 0;
            end

            % Copy base optimization problem (has structure, variables, base constraints)
            opti = obj.opti_base.copy();

            % Get references to variables from copied problem
            U = obj.U_var;
            X = obj.X_var;

            % Set parameter values
            opti.set_value(obj.x0_param, x);
            opti.set_value(obj.xref_param, xref_horizon);

            % Build cost function
            cost = 0;
            for k = 1:obj.N
                % State tracking error
                x_err = X(:,k) - obj.xref_param(k,:)';
                cost = cost + x_err' * obj.Q * x_err;

                % Control effort
                cost = cost + U(:,k)' * obj.R * U(:,k);
            end

            % Terminal cost
            x_err = X(:,obj.N+1) - obj.xref_param(obj.N+1,:)';
            cost = cost + x_err' * obj.Q * x_err;

            % Add CBF constraints dynamically (only if obstacles present)
            if n_obstacles > 0
                if obj.use_slack
                    % Create slack variables for soft constraints
                    S = opti.variable(n_obstacles, obj.N);

                    % Slack must be non-negative (element-wise)
                    for i = 1:n_obstacles
                        for k = 1:obj.N
                            opti.subject_to(S(i,k) >= 0);
                        end
                    end

                    % Add CBF constraints with slack
                    for k = 1:obj.N
                        v = U(1,k);
                        omega = U(2,k);

                        h = ranges - obj.d_safe;
                        h_dot = -v * cos(bearings) - ranges .* omega .* sin(bearings);

                        for i = 1:n_obstacles
                            opti.subject_to(h_dot(i) + obj.alpha_cbf * h(i) + S(i,k) >= 0);
                        end
                    end

                    % Add slack penalty to cost
                    cost = cost + obj.slack_penalty * sum(sum(S.^2));
                else
                    % Hard CBF constraints
                    for k = 1:obj.N
                        v = U(1,k);
                        omega = U(2,k);

                        h = ranges - obj.d_safe;
                        h_dot = -v * cos(bearings) - ranges .* omega .* sin(bearings);

                        for i = 1:n_obstacles
                            opti.subject_to(h_dot(i) + obj.alpha_cbf * h(i) >= 0);
                        end
                    end
                end
            end

            % Set the objective function
            opti.minimize(cost);

            % Set initial guess (warm start)
            opti.set_initial(U, obj.last_u);

            % Solve
            try
                sol = opti.solve();
                u_opt = sol.value(U);
                obj.last_u = u_opt;  % Store for warm start
                u = u_opt(:,1);  % Return first control
            catch
                warning('NMPC optimization failed, using last solution');
                u = obj.last_u(:,1);
            end
        end

        function reset(obj)
            obj.last_u = zeros(2, obj.N);
        end

        function info(obj)
            fprintf('============== MPC Information ===============\n');
            fprintf('  Horizon Length (N):        %d steps\n', obj.N);
            fprintf('  Time Step (dt):            %.3f s\n', obj.dt);
            fprintf('  State Weights (Q):         [%.2f, %.2f, %.2f]\n', diag(obj.Q)');
            fprintf('  Control Weights (R):       [%.2f, %.2f]\n', diag(obj.R)');
            fprintf('  Linear velocity:           [%.2f, %.2f] m/s\n', obj.v_min, obj.v_max);
            fprintf('  Angular velocity:          [%.2f, %.2f] rad/s\n', obj.omega_min, obj.omega_max);
            fprintf('  Safety Radius (d_safe):    %.2f m\n', obj.d_safe);
            fprintf('  Aggressiveness (α):        %.2f\n', obj.alpha_cbf);
            fprintf('  Constraint Range:          %.2f m\n', obj.constraint_range);
            fprintf('  Field of View:             %.1f deg\n', rad2deg(obj.scan_fov));
            fprintf('  Scan Downsample Factor:    %d\n', obj.scan_downsample);
            fprintf('  Use Slack Variables:       %s\n', string(obj.use_slack));
            if obj.use_slack
                fprintf('  Slack Penalty:             %.1f\n', obj.slack_penalty);
            end
            fprintf('  Max Iterations:            %d\n', obj.solver_options.ipopt.max_iter);
            fprintf('================================================\n\n');
        end
    end
end
