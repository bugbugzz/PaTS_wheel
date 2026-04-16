function [lambda] = lagrangeMultipliers(q,ddq,t,Cq_f,M)
    g = 9.81;
    Qa = zeros(size(q));

    % 1. Internal Gravity (Mass of the TPU links)
    for i = 2:3:length(q)
        m_body = M(i,i);
        Qa(i) = -m_body * g;
    end

    % 2. EXTERNAL ROBOT LOAD (The 1.3kg Robot)
    % apply the robot weight to the chassis pins
    W_robot = 1.3 * g; 
    Qa(2)  = Qa(2)  - (W_robot * 0.50); % 50% on central hub
    Qa(5)  = Qa(5)  - (W_robot * 0.25); % 25% on claw support
    Qa(11) = Qa(11) - (W_robot * 0.25); % 25% on pad support

    % 3. GROUND REACTION (The Step holding the claw up)
    Qa(8) = W_robot; 

    lambda = Cq_f(q,t)' \ (Qa - M * ddq);
end
