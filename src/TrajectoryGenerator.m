function traj = TrajectoryGenerator(tse, tsci, tscf, tceg, tces, k)

% Setting the time durations for each movement phase  
% - The robot moves in 8 segments, each with a different time allocation  
time = [ 10;    % Moving to the grasp standoff position  
         3;     % Moving down to grip the object  
         0.625; % Closing the gripper (fixed duration)  
         3;     % Lifting the object back to standoff  
         10;    % Moving to the final standoff position  
         3;     % Moving down to place the object  
         0.625; % Opening the gripper (fixed duration)  
         3;     % Moving back up to the final standoff  
       ];

% Choosing a time scaling method for smooth motion  
% - 3: Cubic scaling (moderate acceleration)  
% - 5: Quintic scaling (smoother motion)  
method = 5;

% Choosing the type of trajectory to follow  
% - 1: Cartesian trajectory (straight-line movement)  
% - 2: Screw trajectory (rotation included)  
traj_type = 1;

% Defining the key configurations during movement  
% Each row represents a step in the trajectory  
configCell = { tse, 0;         % Start at the initial position  
               tsci*tces, 0;   % Move down to the object  
               tsci*tceg, 1;   % Close the gripper  
               tsci*tceg, 1;   % Lift the object back up  
               tsci*tces, 1;   % Move to the final standoff  
               tscf*tces, 1;   % Move down to place the object  
               tscf*tceg, 0;   % Open the gripper  
               tscf*tceg, 0;   % Move back up to the final standoff  
               tscf*tces, 0; }; 

traj = []; % Initializing the trajectory  

% Looping through each movement phase  
for i = 1:8
    % Determine the number of trajectory points based on time  
    N = round(time(i) * k * 100);  
    start = configCell{i,1};  
    finish = configCell{i+1,1};  
    tf = time(i);  

    % Generate the trajectory using the chosen method  
    if traj_type == 1
        tempCell = CartesianTrajectory(start, finish, tf, N, method);
    elseif traj_type == 2
        tempCell = ScrewTrajectory(start, finish, tf, N, method);
    end

    % Convert the trajectory to row format and append to final trajectory  
    for j = 1:length(tempCell)
        traj = [ traj; se3ToRow(tempCell{j}), configCell{i,2} ];
    end
end

end  
