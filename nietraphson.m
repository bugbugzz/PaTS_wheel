function [q1] = nietraphson(q1, t, C_f, Cq_f, tol_I, tol_step, maxI)
%{
This function performs Newton Raphson to find the configuration at time t.
It does NOT contain initialization logic. It just solves f(q) = 0.
%}

    % Initialize
    NoI = 0;
    % q0 = q1;
    q0 = zeros(size(q1));

    
    % Evaluate error
    
    % Newton-Raphson Loop
    % while (norm(F) > tol_I) && (NoI < maxI)
    while (norm(C_f(q1, t)) > tol_I) && (norm(q1 - q0) > tol_step) && (NoI < maxI)    % FILL IN the three conditions to determine whether a Newton Raphson iteration must be performed  
        q0 = q1; %try
        F = C_f(q0, t);
        J = Cq_f(q0, t);        % Calculate Jacobian
        q1 = q0 - J \ F;        % Update position dq is J inverse q 
        NoI = NoI + 1;
    end
    
    % Check if failed
    if NoI >= maxI
        % Optional: Print warning but don't crash immediately if you want to see partial results
        warning(['Max NoIations reached at t = ' num2str(t)]);
     % 2. Did we stop moving but the answer is still wrong? (Stagnation)
    elseif norm(C_f(q1, t)) > tol_I 
        error('Solver stagnated: Step size is small, but error is still large.')
    end
end