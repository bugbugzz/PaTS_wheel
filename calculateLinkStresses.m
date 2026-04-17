function [sigma_total, sigma_axial, sigma_bending] = calculateLinkStresses(Qc, b, h_array, L_vals)
    [~, num_steps] = size(Qc);
    sigma_axial = zeros(5, num_steps);
    sigma_bending = zeros(5, num_steps);
    sigma_total = zeros(5, num_steps);
    
    % Approximate lever arms (distance from CG to the hinge for each body)
    % Bodies: [Coupler, Claw Support, Claw Tip, Pad Support, Pad Tip]
    L_arm = [L_vals(4)/2, L_vals(5)/2, L_vals(6)/2, L_vals(8)/2, L_vals(6)/2]; 
    
    for i = 1:5
        % Calculate properties specific to a hinge's thickness
        h_current = h_array(i);
        A = b * h_current;                 % Area [m^2]
        I_area = (b * h_current^3) / 12;   % Moment of Inertia [m^4]
        y_max = h_current / 2;             % Distance to extreme fiber

        idx = 3*(i-1)+1 : 3*i;
        Fx = Qc(idx(1), :);
        Fy = Qc(idx(2), :);
        
        % The total force acting on the body
        F_mag = sqrt(Fx.^2 + Fy.^2);
        
        % 1. Loading (Axial) Stress: Force pulling the link apart
        sigma_axial(i, :) = F_mag / A;
        
        % 2. Bending Stress: Force * Lever Arm to the hinge
        M_real = F_mag * L_arm(i); 
        sigma_bending(i, :) = (M_real * y_max) / I_area;
        
        % Total Internal Stress at the weakest point
        sigma_total(i, :) = sigma_axial(i, :) + sigma_bending(i, :);
    end
end