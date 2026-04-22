function printSystemParameters(L_vals, b_flex, h_flex, L_notch, scale, s_axial, s_bend, SF)
    % prints all the model specs and safety factors to the console
    
    fprintf('\nPaTS-wheel 3d model parameters (scaled %.2fx)\n', scale);
    fprintf('--------------------------------------------------\n');
    
    % print out the lengths
    fprintf('Kinematic skeleton lengths (m):\n');
    fprintf('  L1 (claw pivot offset):  %.4f\n', L_vals(1));
    fprintf('  L2 (coupler right beam): %.4f\n', L_vals(2));
    fprintf('  L3 (pad pivot offset):   %.4f\n', L_vals(3));
    fprintf('  L4 (coupler left beam):  %.4f\n', L_vals(4));
    fprintf('  L5 (claw support link):  %.4f\n', L_vals(5));
    fprintf('  L6 (claw base link):     %.4f\n', L_vals(6));
    fprintf('  L7 (pad base link):      %.4f\n', L_vals(7));
    fprintf('  L8 (pad support link):   %.4f\n', L_vals(8));
    
    % flexure data
    fprintf('\nFlexure hinge parameters:\n');
    fprintf('  Type:                    Leaf spring\n');
    fprintf('  Thicknesses (h):         [%.1f, %.1f, %.1f, %.1f, %.1f] mm\n', h_flex * 1000);
    fprintf('  Extrusion width (b):     %.4f m (%.1f mm)\n', b_flex, b_flex * 1000);
    fprintf('  Bending length:          %.4f m (%.1f mm)\n', L_notch, L_notch * 1000);
    
    % targets
    fprintf('\nPerformance targets:\n');
    fprintf('  Footprint width:         %.1f mm\n', (L_vals(1) + L_vals(3)) * 1000);
    fprintf('  Obstacle height:         180 mm\n');
    fprintf('  Robot weight support:    1.3 kg\n');
    
    % run through the safety factors
    fprintf('\nStress report: 1.3kg robot load\n');
    fprintf('%-25s | %-12s | %-12s | %-8s\n', 'Hinge Node', 'Axial (MPa)', 'Bending (MPa)', 'SF');
    fprintf('----------------------------------------------------------------------\n');
    node_names = {'Coupler hinge','Claw base hinge','Claw tip hook','Pad base hinge', 'Pad tip hinge'};
    for i = 1:5
        fprintf('%-25s | %-12.4f | %-12.4f | %-8.2f\n', ...
                node_names{i}, max(s_axial(i,:))/1e6, max(s_bend(i,:))/1e6, SF(i));
    end
    fprintf('\nSimulation finished.\n');
end