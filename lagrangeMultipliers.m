function [lambda] = lagrangeMultipliers(q,ddq,t,Cq_f,M)
    g = 9.81;
    Qa = zeros(size(q));
    
    % --- CLIMBING SIMULATION (1.3kg Robot) ---
    % The weight of the robot pulls down on the chassis pins (Y-indices: 2, 5, 11)
    W_robot = 1.3 * g; % ~12.75 Newtons
    
    Qa(2)  = -(W_robot * 0.50); % 50% load on central hub
    Qa(5)  = -(W_robot * 0.25); %25% load on claw support pin
    Qa(11) = -(W_robot * 0.25); % 25% load on pad support pin

    % The Claw Tip (Body 3, Y-index: 8) is hooked on the 18cm step.
    % It pushes UP against the robot's weight to lift it.
    Qa(8) = W_robot; 

    lambda = Cq_f(q,t)' \ (Qa - M * ddq);
end