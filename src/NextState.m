function [updated_configuration] = NextState(current_configuration, control_velocity, control_velocity_limits, time_step)
% This function computes the next state of the robot given its current configuration, 
% control velocity inputs, velocity limits, and the time step.

% Extracting the current states of different components
chassis_state = current_configuration(1:3); % Chassis position and orientation
joint_state = current_configuration(4:8); % Joint angles of the arm
wheel_state = current_configuration(9:12); % Wheel encoder readings

% Ensuring velocity inputs are column vectors for consistency
control_velocity = control_velocity(:);
control_velocity_limits = control_velocity_limits(:);

% Checking if any velocity exceeds its respective limit
exceeded_velocities = abs(control_velocity) > abs(control_velocity_limits);

% If a velocity exceeds its limit, we scale it down to match the limit
control_velocity(exceeded_velocities) = control_velocity(exceeded_velocities) ./ ...
                                        abs(control_velocity(exceeded_velocities)) .* ...
                                        control_velocity_limits(exceeded_velocities);

% Extracting joint and wheel velocities from control input
joint_velocity = control_velocity(5:9);
wheel_velocity = control_velocity(1:4);

% Making sure these are row vectors for calculations
joint_velocity = joint_velocity(:)'; 
wheel_velocity = wheel_velocity(:)'; 

% Updating joint states based on computed joint velocities
joint_state = joint_state + joint_velocity * time_step;

% Updating wheel states based on computed wheel velocities
wheel_state = wheel_state + wheel_velocity * time_step;

% Computing the new chassis state using kinematics
phi = chassis_state(1); % Extracting current chassis orientation

% Updating chassis position using a transformation matrix
% This accounts for the movement caused by wheel velocities
chassis_state(:) = chassis_state(:) + [1, 0, 0;
                                       0, cos(phi), -sin(phi);
                                       0, sin(phi), cos(phi)] * ...
                                       [-0.0308,  0.0308,  0.0308, -0.0308;
                                        ones(1,4) * 0.0119;
                                        -0.0119, 0.0119, -0.0119, 0.0119] * ...
                                        wheel_velocity(:) * time_step;

% Concatenating the updated states to form the new configuration
updated_configuration = [chassis_state(:)' , joint_state(:)', wheel_state(:)'];

end
