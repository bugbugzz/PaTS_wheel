function [sigma_total, sigma_axial, sigma_bending, SF] = calculateLinkStresses(Qc, b, h, yield_strength)
    [~, num_steps] = size(Qc);
    sigma_axial = zeros(5, num_steps);
    sigma_bending = zeros(5, num_steps);
    sigma_total = zeros(5, num_steps);
    
    % Section Properties for a Rectangular Flexure
    A = b * h;              % Area [m^2]
    I_area = (b * h^3) / 12; % Moment of Inertia [m^4]
    y_max = h / 2;          % Distance to extreme fiber
    
    for i = 1:5
        idx = 3*(i-1)+1 : 3*i;
        Fx = Qc(idx(1), :);
        M  = Qc(idx(3), :);
        
        % Component 1: Loading (Axial) Stress
        sigma_axial(i, :) = abs(Fx / A);
        
        % Component 2: Bending Stress
        sigma_bending(i, :) = (abs(M) * y_max) / I_area;
        
        % Total Stress
        sigma_total(i, :) = sigma_axial(i, :) + sigma_bending(i, :);
    end
    
    % Safety Factor based on peak total stress
    SF = yield_strength ./ max(sigma_total, [], 2);
end