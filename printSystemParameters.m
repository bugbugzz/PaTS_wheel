function printSystemParameters(L_vals, b_flex, h_flex, L_notch, scale, s_axial, s_bend, SF)
    % This function prints all 3D modeling dimensions and the detailed
    % Stress/Safety Factor report to the console.
    
    fprintf('\n============================================================\n');
    fprintf('   PaTS-WHEEL 3D MODELING PARAMETERS (Scaled %.2fx)\n', scale);
    fprintf('============================================================\n');
    
    % 1. Linkage Lengths (Center-to-Center of Hinges)
    fprintf('\n--- KINEMATIC SKELETON DIMENSIONS (m) ---\n');
    fprintf('L1 (Claw Pivot Offset):    %.4f\n', L_vals(1));
    fprintf('L2 (Coupler Right Beam):   %.4f\n', L_vals(2));
    fprintf('L3 (Pad Pivot Offset):     %.4f\n', L_vals(3));
    fprintf('L4 (Coupler Left Beam):    %.4f\n', L_vals(4));
    fprintf('L5 (Claw Support Link):    %.4f\n', L_vals(5));
    fprintf('L6 (Claw Base Link):       %.4f\n', L_vals(6));
    fprintf('L7 (Pad Base Link):        %.4f\n', L_vals(7));
    fprintf('L8 (Pad Support Link):     %.4f\n', L_vals(8));
    
    % 2. Flexure Hinge Design (Leaf Springs)
    fprintf('\n--- FLEXURE HINGE (LEAF SPRING) PARAMETERS ---\n');
    fprintf('Flexure Type:         Leaf Flexure (Flat Leaf Spring)\n');
    fprintf('Thicknesses (h):      [%.1f, %.1f, %.1f, %.1f, %.1f] mm\n', h_flex * 1000);
    fprintf('Width/Depth (b):      %.4f m (%.1f mm)\n', b_flex, b_flex * 1000);
    fprintf('Leaf Length (L_notch):%.4f m (%.1f mm)\n', L_notch, L_notch * 1000);
    
    % 3. Calculated Reach
    fprintf('\n--- PERFORMANCE TARGETS ---\n');
    fprintf('Target Footprint:      %.1f mm\n', (L_vals(1) + L_vals(3)) * 1000);
    fprintf('Obstacle Capability:   180 mm Step\n');
    fprintf('Robot Weight Support:  1.3 kg\n');
    
    % 4. Stress & Safety Factor Report
    fprintf('\n--- STRESS REPORT: 1.3kg Robot Climbing 18cm Step ---\n');
    fprintf('%-25s | %-12s | %-12s | %-8s\n', 'Flexure Node', 'Axial (MPa)', 'Bending (MPa)', 'SF');
    fprintf('----------------------------------------------------------------------\n');
    node_names = {'Coupler Hinge (Index 1)','Claw Base Hinge (Index 2)','Claw Tip Hook (Index 3)','Pad Base Hinge (Index 4)', 'Pad Tip Hinge (Index 5)'};
    for i = 1:5
        fprintf('%-25s | %-12.4f | %-12.4f | %-8.2f\n', ...
                node_names{i}, max(s_axial(i,:))/1e6, max(s_bend(i,:))/1e6, SF(i));
    end
    fprintf('============================================================\n\n');
    
    disp('Simulation Complete.');
end