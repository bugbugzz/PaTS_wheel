function [lambda] = lagrangeMultipliers(q,ddq,t,Cq_f,M)

    % --- NEW CODE START: Constructing Vector Qa ---
    g = 9.81;              % Gravity constant [m/s^2]
    Qa = zeros(size(q));   % Initialize a column vector of Zeros (matches the '0's in your math)
    
    % Loop through each body to apply gravity to the Y-coordinate
    % The diagonal M matrix contains the mass 'm' at indices corresponding to x and y.
    % We want indices 2, 5, 8... (The Y-coordinates)
    for i = 2:3:length(q)
        m_body = M(i,i);     % Extract mass 'm' from the diagonal M matrix
        Qa(i) = -m_body * g; % Apply -mg to the Y-row
    end
    % --- NEW CODE END ---

    % Now the equation works because Qa exists:
    lambda = Cq_f(q,t)' \ (Qa - M * ddq);  

end

% function [lambda] = lagrangeMultipliers(q,ddq,t,Cq_f,M)
%     g = 9.81;
%     Qa = zeros(size(q));
% 
%     % 1. ROBOT LOAD (The Lifting Phase)
%     % Put the 1.3kg robot weight on the CLAW (Body 3, Y-index is 8)
%     % This simulates the robot "hanging" from the step.
%     m_robot = 1.3;
%     Qa(8) = -(m_robot * g); % ~12.75 Newtons
% 
%     % 2. MOTOR TORQUE
%     % Simulate the motor pushing the Pad (Body 5, X-index is 13)
%     % This provides the "shove" needed to clear the 18cm step.
%     Qa(13) = -15; 
% 
%     lambda = Cq_f(q,t)' \ (Qa - M * ddq);
% end