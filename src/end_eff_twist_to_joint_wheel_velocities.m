function [V, Je] = end_eff_twist_to_joint_wheel_velocities(theta, Ve)

% Ensure theta and Ve are column vectors  
theta = theta(:);
Ve = Ve(:);

% Pseudo-inverse of H0 (used to compute wheel velocities)  
H0_inv = [  0.0000   -0.0000   -0.0000    0.0000;
           -0.0000    0.0000    0.0000   -0.0000;
           -0.0308    0.0308    0.0308   -0.0308;
            0.0119    0.0119    0.0119    0.0119;
           -0.0119    0.0119   -0.0119    0.0119;
            0         0         0         0      ];

% Transformation from the end-effector frame {e} to the arm base frame {o}  
Moe = [ 1 0 0 0.033; 
        0 1 0 0;
        0 0 1 0.700;
        0 0 0 1 ];

% Screw axes for each joint (expressed in the body frame)  
B = [    0         0         0         0         0;
         0   -1.0000   -1.0000   -1.0000         0;
    1.0000         0         0         0    1.0000;
         0   -0.5076   -0.3526   -0.2176         0;
    0.0330         0         0         0         0;
         0         0         0         0         0  ];

% Transformation from the chassis frame {b} to the arm base frame {o}  
Tbo = [  1.0000         0         0    0.1662;
              0    1.0000         0         0;
              0         0    1.0000    0.0026;
              0         0         0    1.0000 ];

% Compute the transformation from {o} to {e} using forward kinematics  
Toe = FKinBody(Moe, B, theta);

% Compute the transformation from {b} to {e}  
Tbe = Tbo * Toe;
Teb = TransInv(Tbe); % Get the inverse transformation  

% Compute the adjoint transformation to map twists from {b} to {e}  
ADJ_Teb = Adjoint(Teb);

% Compute the base Jacobian (how the wheels affect motion)  
Jbase = ADJ_Teb * H0_inv;

% Compute the arm Jacobian (how the joints affect motion)  
Jarm = JacobianBody(B, theta);

% Combine the two Jacobians to get the full Jacobian for the system  
Je = [ Jbase Jarm ];

% Set a tolerance to prevent numerical instability in the pseudo-inverse  
% Any values below this threshold are treated as zero before computing pinv  
tolerance = 1e-6;
V = pinv(Je, tolerance) * Ve;

% Apply a scaling matrix to control individual velocity components  
K = eye(9) .* [ 1 1 1 1 1 1 1 1 1];
V = K * V;

end  
