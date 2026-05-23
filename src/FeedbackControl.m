function [V, Xe, integral] = FeedbackControl(X, Xd, Xdn, Kp, Ki, integral, timestep)
    % Compute error between current and desired end-effector poses
    % I first find the transformation from current to desired, then extract error
    Xe = se3ToVec(MatrixLog6(TransInv(X) * Xd));
    
    % Calculate feedforward twist based on change in desired pose over time
    % This helps anticipate motion rather than just correcting error
    Vd = se3ToVec(MatrixLog6(TransInv(Xd) * Xdn) / timestep);
    
    % Compute the adjoint representation to transform Vd into the error frame
    ADJ_xbd = Adjoint(TransInv(X) * Xd);
    
    % Update integral term for PI control
    integral = integral + Xe * timestep;
    
    % Implement the feedback control law using feedforward + PI terms
    % The first term accounts for desired motion, the second for proportional correction,
    % and the third integrates past errors to reduce steady-state error
    V = ADJ_xbd * Vd + Kp * Xe + Ki * integral;
    
    % Alternative: Only use PI control without feedforward
    % V = Kp * Xe + Ki * integral;
end
