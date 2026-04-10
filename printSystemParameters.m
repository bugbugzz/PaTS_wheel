function printSystemParameters(L_vals, b_flex, h_flex, scale)
    % This function prints the 3D modeling dimensions for the PaTS-Wheel
    
    fprintf('\n============================================================\n');
    fprintf('   PaTS-WHEEL 3D MODELING PARAMETERS (Scaled %.2fx)\n', scale);
    fprintf('============================================================\n');
    
    % 1. Linkage Lengths (Center-to-Center of Hinges)
    fprintf('\n--- LINKAGE DIMENSIONS (m) ---\n');
    fprintf('Coupler (Link 1):     %.4f\n', L_vals(1));
    fprintf('Inner Supports:       %.4f\n', L_vals(2)); % a2/b3 distances
    fprintf('Claw Body:            %.4f\n', L_vals(3));
    fprintf('Pad Body:             %.4f\n', L_vals(4));
    
    % 2. Flexure Hinge Design (The "Nodes")
    fprintf('\n--- FLEXURE HINGE (NOTCH) PARAMETERS ---\n');
    fprintf('Flexure Type:         Notch Flexure (Living Hinge)\n');
    fprintf('Thickness (h):        %.4f m (%.1f mm)\n', h_flex, h_flex*1000);
    fprintf('Width/Depth (b):      %.4f m (%.1f mm)\n', b_flex, b_flex*1000);
    fprintf('Suggested Notch Len:  0.0030 m (3.0 mm) - Standard Notch\n');
    
    % 3. Calculated Reach
    fprintf('\n--- PERFORMANCE TARGETS ---\n');
    fprintf('Target Wheel Diameter: %.1f mm\n', 100 * scale);
    fprintf('Obstacle Capability:   180 mm Step\n');
    fprintf('Robot Weight Support:  1.3 kg\n');
    fprintf('============================================================\n');
end