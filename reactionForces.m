function [Qc] = reactionForces(q, t, lambda, L_vals, Cq_f)
%{
This function calculates the generalized reaction forces (Qc) 
acting on the center of mass of every body in the mechanism.

INPUT:
    q       The coordinates at the current time instance (15x1)
    t       Current time instance
    lambda  The lagrange multipliers at the current time instance (15x1)
    L_vals  The lengths of the bars (not strictly needed for this matrix approach)
    Cq_f    Function of the jacobian of the constraints
OUTPUT: 
    Qc      The generalized reaction forces [Fx; Fy; M] for all 5 bodies (15x1)
%}

% 1. Calculate the Jacobian matrix at the current time and position
Cq = Cq_f(q, t);

% 2. Calculate the Generalized Constraint Forces
% Formula: Qc = -Cq^T * lambda
% This automatically maps the joint forces (lambda) to the X, Y, and Torque 
% acting on the center of mass of each of the 5 bodies.
Qc = -Cq' * lambda;

end