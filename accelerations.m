function [ddq] = accelerations(q, dq, t, Cq_f, Cqt_f, Ctt_f, Cqqdq_f)
%{
This function calculates the accelerations for the mechanism. 
Because it uses generic matrix operations, it automatically scales 
to handle the 15-element vectors (5 bodies) of the PaTS-Wheel system.

INPUT:
    q           Position vector (15x1)
    dq          Velocity vector (15x1)
    t           Current time scalar
    Cq_f        The Jacobian matrix (15x15)
    Cqt_f       The time derivative of the Jacobian (15x15)
    Ctt_f       The second time derivative of the constraints (15x1)
    Cqqdq_f     The derivative of (Cq*dq) with respect to coordinates (15x15)

OUTPUT:
    ddq         Acceleration vector (15x1)
%}

% 1. Calculate the Right-Hand Side Acceleration Vector (gamma)
% Formula: gamma = - (Cq*dq)q * dq - 2 * Cqt * dq - Ctt   
% (Ref: Flexible Multibody Dynamics Reader, Eq 3.7)
gamma = -Cqqdq_f(q, dq) * dq - 2 * Cqt_f(q, t) * dq - Ctt_f(q, t); 

% 2. Solve for Accelerations (ddq)
% Formula: Cq * ddq = gamma  =>  ddq = Cq^-1 * gamma   
% (Ref: Flexible Multibody Dynamics Reader, Eq 3.9)
% The backslash operator (\) is used to solve the linear system efficiently.
ddq = Cq_f(q, t) \ gamma;

end