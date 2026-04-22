function [s_bend_max, s_axial_max, s_total_max, SF] = calculateLinkStresses(qSol, Qc, E_tpu, b_flex, h_array, D_array, yield_strength)
    % Extracts positions and forces to calculate JPE Bending and Cauchy Axial stress
    
    [~, num_steps] = size(qSol);
    s_bend = zeros(5, num_steps);
    s_axial = zeros(5, num_steps);
    s_total = zeros(5, num_steps);
    
    % Step 1: Extract the global angles for all 5 links
    th = qSol([3, 6, 9, 12, 15], :);
    
    % Step 2: Calculate the Relative Rotations (Ry) for JPE Bending
    Ry = zeros(5, num_steps);
    Ry(1, :) = abs(th(3, :) - th(1, :)); % Coupler hinge
    Ry(2, :) = abs(th(3, :) - th(2, :)); % Claw base hinge
    Ry(3, :) = 0;                        % Claw tip (solid hook, no relative flexure bending)
    Ry(4, :) = abs(th(5, :) - th(1, :)); % Pad base hinge
    Ry(5, :) = abs(th(5, :) - th(4, :)); % Pad tip hinge
    
    for i = 1:5
        % ----------------------------------------------------
        % A. BENDING STRESS (JPE Innovations Framework)
        % ----------------------------------------------------
        beta = h_array(i) / D_array(i);
        S = 0.58 * sqrt(beta);
        s_bend(i, :) = E_tpu * S * Ry(i, :);
        
        % ----------------------------------------------------
        % B. AXIAL STRESS (Cauchy Normal Stress: F / A)
        % ----------------------------------------------------
        % Find the indices in Qc for this specific body's x and y forces
        idx_x = 3*(i-1) + 1;
        idx_y = 3*(i-1) + 2;
        
        Fx_global = Qc(idx_x, :);
        Fy_global = Qc(idx_y, :);
        theta_i = th(i, :);
        
        % Project the global forces onto the local axis of the link to get pure pull/push
        F_axial = Fx_global .* cos(theta_i) + Fy_global .* sin(theta_i);
        
        % Calculate area at the weakest point (waist of the notch)
        Area = b_flex * h_array(i);
        s_axial(i, :) = abs(F_axial) / Area;
        
        % ----------------------------------------------------
        % C. SUPERPOSITION (Total Stress)
        % ----------------------------------------------------
        s_total(i, :) = s_bend(i, :) + s_axial(i, :);
    end
    
    % Step 3: Extract the absolute maximums across the entire time simulation
    s_bend_max = max(s_bend, [], 2);
    s_axial_max = max(s_axial, [], 2);
    s_total_max = max(s_total, [], 2);
    
    % Calculate Safety Factor based on the combined total stress
    SF = yield_strength ./ s_total_max;
end