function Tse = robot_state_to_end_eff_state(robot_config)
%ROBOT_STATE_TO_END_EFF_STATE Computes the youBot end-effector pose in space frame.
% robot_config order: [phi, x, y, arm joints 1-5, wheel angles 1-4].

phi = robot_config(1);
x = robot_config(2);
y = robot_config(3);
theta = robot_config(4:8);

Tsb = [cos(phi), -sin(phi), 0, x;
       sin(phi),  cos(phi), 0, y;
       0,         0,        1, 0.0963;
       0,         0,        0, 1];

Tbo = [1, 0, 0, 0.1662;
       0, 1, 0, 0;
       0, 0, 1, 0.0026;
       0, 0, 0, 1];

Moe = [1, 0, 0, 0.033;
       0, 1, 0, 0;
       0, 0, 1, 0.700;
       0, 0, 0, 1];

B = [0,       0,       0,       0,       0;
     0,      -1.0000, -1.0000, -1.0000,  0;
     1.0000,  0,       0,       0,       1.0000;
     0,      -0.5076, -0.3526, -0.2176,  0;
     0.0330,  0,       0,       0,       0;
     0,       0,       0,       0,       0];

Toe = FKinBody(Moe, B, theta);
Tse = Tsb * Tbo * Toe;
end
