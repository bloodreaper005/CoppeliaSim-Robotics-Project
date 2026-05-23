function [robot_states, error_accum, manipulability] = main(Tsci, Tscf, current_robot_config, Tse, Kp, Ki, output_dir, enable_plots, debug)
    if nargin < 7 || isempty(output_dir)
        output_dir = pwd;
    end
    if nargin < 8 || isempty(enable_plots)
        enable_plots = true;
    end
    if nargin < 9 || isempty(debug)
        debug = false;
    end
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    % Define the transformation matrices for the gripper's states  
    % - Tces: When the gripper is open  
    % - Tceg: When the gripper is closed  
    Tces = [-1 0 0 0; 
            0 1 0 0; 
            0 0 -1 0.1; 
            0 0 0 1];

    Tceg = [-1 0 0 0; 
            0 1 0 0; 
            0 0 -1 -0.01; 
            0 0 0 1];

    % Define step size for trajectory  
    k = 1;  
    timestep = 0.01 / k;  

    % Define speed limits for the robot  
    control_velocity_limits = [9,9,9,9,9,9,9,9,9];

    % Define joint movement limits (to prevent unrealistic angles)  
    joint_limits = [-pi, pi;  
                    -pi, pi;
                    -pi, pi;
                    -pi, pi;
                    -pi, pi];

    % Generate the end-effector’s movement path  
    end_eff_trajectory = TrajectoryGenerator(Tse, Tsci, Tscf, Tceg, Tces, k);

    % Figure out how many iterations we need  
    num_iterations = size(end_eff_trajectory, 1) - 1;
    
    % Store robot state and error values for each step  
    robot_states = zeros(num_iterations + 1, 13);
    error_accum = zeros(num_iterations, 6);
    
    % Store manipulability factors  
    manipulability_Aw = zeros(num_iterations, 1);
    manipulability_Av = zeros(num_iterations, 1);
    
    % Set the initial robot state  
    robot_states(1, :) = current_robot_config;
    actual_robot_config = current_robot_config(1:12);
    integral = zeros(6,1); % Error integral starts at zero  

    % Loop through each step in the trajectory  
    for i = 1:num_iterations
        % Get the gripper state from the trajectory  
        gripper_state = end_eff_trajectory(i, 13);

        % Get the current and desired end-effector positions  
        actual_end_effector = robot_state_to_end_eff_state(actual_robot_config(1:12));
        desired_end_effector = row2se3(end_eff_trajectory(i, 1:12));
        next_desired_end_effector = row2se3(end_eff_trajectory(i+1, 1:12));

        % Calculate the required twist (motion command) for the end-effector  
        [end_effector_twist, Xe, integral] = FeedbackControl(actual_end_effector, desired_end_effector, next_desired_end_effector, Kp, Ki, integral, timestep);

        % Store the error  
        error_accum(i, :) = Xe(:)';

        % Compute the joint and wheel velocities needed to achieve this twist  
        [control_velocity, Je] = end_eff_twist_to_joint_wheel_velocities(actual_robot_config(4:8), end_effector_twist);

        % Ensure the Jacobian has the expected size (6x9)  
        if size(Je,1) ~= 6 || size(Je,2) ~= 9
            error("Jacobian Je has incorrect dimensions: %dx%d", size(Je,1), size(Je,2));
        end

        % Compute manipulability factors (to measure how easily the robot can move)  
        Aw = Je(1:3, :); % Angular velocity Jacobian  
        Av = Je(4:6, :); % Linear velocity Jacobian  
        singular_values_Aw = svd(Aw);
        singular_values_Av = svd(Av);

        % Prevent division by zero by ensuring we never divide by extremely small values  
        min_Aw = max(singular_values_Aw(end), 1e-6);
        min_Av = max(singular_values_Av(end), 1e-6);

        % Compute the manipulability factor (making sure it's always ≥ 1)  
        manipulability_Aw(i) = sqrt(singular_values_Aw(1)) / sqrt(min_Aw);
        manipulability_Av(i) = sqrt(singular_values_Av(1)) / sqrt(min_Av);

        if debug
            fprintf('Jacobian Je at iteration %d:\n', i);
            disp(Je);
        end

        % Update the robot's state based on the new movement commands  
        actual_robot_config = NextState(actual_robot_config, control_velocity, control_velocity_limits, timestep);

        % Enforce joint limits (making sure joints don’t exceed their allowed range)  
        for j = 4:8
            actual_robot_config(j) = max(min(actual_robot_config(j), joint_limits(j-3,2)), joint_limits(j-3,1));
        end

        % Save the updated robot state  
        robot_states(i+1, :) = [actual_robot_config, gripper_state];
    end

    % Save the robot’s state and error history for later analysis  
    csvwrite(fullfile(output_dir, 'robot_states_case.csv'), robot_states);
    csvwrite(fullfile(output_dir, 'error.csv'), error_accum);
    fprintf('Saved robot states and errors to %s\n', output_dir);

    manipulability = struct('angular', manipulability_Aw, 'linear', manipulability_Av);

    % Generate a time vector that correctly matches the recorded data  
    time_vector = linspace(timestep, (num_iterations - 1) * timestep, num_iterations - 1);

    if enable_plots
        % --- Plot Manipulability Factors ---  
        figure;
        subplot(2,1,1);
        plot(time_vector, manipulability_Aw(2:end), 'b', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('\mu_1(A_w)');
        title('Manipulability Factor (Angular)');

        subplot(2,1,2);
        plot(time_vector, manipulability_Av(2:end), 'r', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('\mu_1(A_v)');
        title('Manipulability Factor (Linear)');

        % --- Plot Error Convergence (Xerr) ---  
        figure;
        for j = 1:6
            subplot(3,2,j);
            plot(time_vector, error_accum(2:end,j), 'LineWidth', 2);
            xlabel('Time (s)');
            ylabel(['Xerr_', num2str(j)]);
            title(['Error Element Xerr_', num2str(j)]);
        end
    end

end  
