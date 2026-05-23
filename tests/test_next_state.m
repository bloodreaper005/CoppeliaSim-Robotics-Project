% Test script for NextState function
project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(project_root, 'src'));
output_dir = fullfile(project_root, 'outputs');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% Simulation Parameters
time_step = 0.01;  % 10ms per step
num_steps = 100;   % Run for 1 second
filename = fullfile(output_dir, 'youbot_simulation.csv');  % Output file

% Initial Configuration [chassis (3) + arm (5) + wheel (4)]
initial_configuration = [0, 0, 0, ...   % Chassis: (phi, x, y)
                         0, 0, 0, 0, 0, ... % Arm joints
                         0, 0, 0, 0];   % Wheel angles

% Constant Control Inputs [wheel (4) + arm (5)]
wheel_speeds = [10, 10, 10, 10];  % Set wheel speeds (rad/s)
joint_speeds = [0, 0, 0, 0, 0];   % Keep arm joints stationary
control_velocity = [wheel_speeds, joint_speeds]; 

% Velocity Limits (Assumed for safety)
wheel_velocity_limits = [15, 15, 15, 15];  % Wheel speed limits (rad/s)
joint_velocity_limits = [1, 1, 1, 1, 1];   % Joint speed limits (rad/s)
control_velocity_limits = [wheel_velocity_limits, joint_velocity_limits];

% Open file for writing
fileID = fopen(filename, 'w');

% Simulation loop (without header)
current_configuration = initial_configuration;
for step = 1:num_steps
    % Compute next state
    current_configuration = NextState(current_configuration, control_velocity, control_velocity_limits, time_step);
    
    % Append "0" for gripper state
    current_configuration_with_gripper = [current_configuration, 0];

    % Write numeric values to CSV (Ensure proper formatting)
    fprintf(fileID, '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', current_configuration_with_gripper);
end

% Close file
fclose(fileID);

disp(['Simulation complete. Data saved to ', filename]);
