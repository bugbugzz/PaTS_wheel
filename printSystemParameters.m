function printSystemParameters(L_vals, b_flex, h_flex, scale)
    fprintf('\n============================================================\n');
    fprintf('   PaTS-WHEEL 3D MODELING PARAMETERS (Scaled %.2fx)\n', scale);
    fprintf('============================================================\n');
    
    % 1. Linkage Dimensions (Updated to match your actual main.m mapping)
    fprintf('\n--- GEOMETRIC DIMENSIONS (m) ---\n');
    fprintf('Claw Pivot Offset (L1): %.4f\n', L_vals(1));
    fprintf('Pad Joint Radius (L2):  %.4f\n', L_vals(2));
    fprintf('Pad Pivot Offset (L3):  %.4f\n', L_vals(3));
    fprintf('Claw Joint Radius (L4): %.4f\n', L_vals(4));
    fprintf('Claw Support (L5):      %.4f\n', L_vals(5));
    fprintf('Tip Body Height (L6):   %.4f\n', L_vals(6));
    fprintf('Pad Support (L8):       %.4f\n', L_vals(8));
    
    % 2. Flexure Hinge Design (The "Nodes")
    fprintf('\n--- FLEXURE HINGE (NOTCH) PARAMETERS ---\n');
    fprintf('Flexure Type:         Notch Flexure (Living Hinge)\n');
    
    % Interleave meters and millimeters correctly for each of the 5 hinges
    thickness_data = [h_flex'; h_flex' * 1000]; % Creates a 2x5 matrix
    node_names = {'Coupler', 'Claw Base', 'Claw Tip', 'Pad Base', 'Pad Tip'};
    
    for i = 1:5
        fprintf('%-12s (h%d):  %.4f m (%.1f mm)\n', ...
            node_names{i}, i, thickness_data(1,i), thickness_data(2,i));
    end
    
    fprintf('Width/Depth (b):      %.4f m (%.1f mm)\n', b_flex, b_flex*1000);
    fprintf('Suggested Notch Len:  0.0080 m (8.0 mm) - Actual Notch\n'); %
    
    fprintf('\n--- strength requirements ---\n');
    fprintf('Robot Weight Support:  1.3 kg\n');
    fprintf('TPU_yield = 20e6 Pa\n');

    fprintf('============================================================\n');
end