function [dq] = velocities(q,t,Cq_f,Ct_f)
%{
This function calculates the velocities.
INPUT:
    q           Position
    t           Time
    Cq_f        The Jacobian
    Ct_f        The derivative of the constraints with respect to time
OUTPUT:
    dq          Velocity
%}

nu = -Ct_f(q, t);      % FILL IN the equation to calculate nu
dq = Cq_f(q, t) \ nu;      % FILL IN the equation to calculate the velocities

end