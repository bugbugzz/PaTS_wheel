function plotMovingLinksAnimation(qSol, time, L_vals, n_frames)
    figure('Name', 'PaTS-Wheel: Connected 5-Body Mechanism', 'Color', 'w');
    hold on; axis equal; grid on;
    xlim([-0.25, 0.25]); ylim([-0.1, 0.3]);
    
    % Define specific colors for clarity
    c_blue = [0 0.447 0.741];   % Link 1 (Coupler/Hub)
    c_red  = [0.85 0.325 0.098]; % Link 2 (Claw Support)
    c_green = [0.466 0.674 0.188]; % Link 3 (Claw Hook)
    c_cyan = [0.301 0.745 0.933]; % Link 4 (Pad Support)
    c_mag  = [0.635 0.078 0.184]; % Link 5 (Pad Foot)

    steps = round(linspace(1, size(qSol, 2), n_frames));
    for i = steps
        cla;
        q = qSol(:,i);
        
        % 1. Chassis/Ground Pins (Stationary)
        Pin_L = [-L_vals(1); 0];
        Pin_R = [ L_vals(3); 0];

        % 2. Body Center Points
        P1 = [q(1); q(2)]; th1 = q(3);
        P2 = [q(4); q(5)]; th2 = q(6);
        P3 = [q(7); q(8)]; th3 = q(9);
        P4 = [q(10);q(11)];th4 = q(12);
        P5 = [q(13);q(14)];th5 = q(15);

        % 3. Calculate Exact Hinge Locations (from main.m constraints)
        H_13 = P1 + [L_vals(4)*cos(th1 + pi - 1.4); L_vals(4)*sin(th1 + pi - 1.4)];
        H_15 = P1 + [L_vals(2)*cos(th1); L_vals(2)*sin(th1)];
        H_23 = P2 + [0.5*L_vals(5)*cos(th2); 0.5*L_vals(5)*sin(th2)];
        H_45 = P4 + [0.5*L_vals(8)*cos(th4); 0.5*L_vals(8)*sin(th4)];

        % --- DRAWING THE CONNECTED LINKS ---
        
        % BLUE: Coupler (Link 1) - Central Spine
        line([H_13(1) H_15(1)], [H_13(2) H_15(2)], 'Color', c_blue, 'LineWidth', 4);
        
        % RED: Claw Support (Link 2) - Pin to Hinge
        line([Pin_L(1) H_23(1)], [Pin_L(2) H_23(2)], 'Color', c_red, 'LineWidth', 4);
        
        % GREEN: Claw Hook (Link 3) - CONNECTS RED TO BLUE
        line([H_23(1) H_13(1)], [H_23(2) H_13(2)], 'Color', c_green, 'LineWidth', 4);
        
        % CYAN: Pad Support (Link 4) - Pin to Hinge
        line([Pin_R(1) H_45(1)], [Pin_R(2) H_45(2)], 'Color', c_cyan, 'LineWidth', 4);
        
        % MAGENTA: Pad Foot (Link 5) - CONNECTS CYAN TO BLUE
        line([H_45(1) H_15(1)], [H_45(2) H_15(2)], 'Color', c_mag, 'LineWidth', 4);

        % Reference ground and 18cm step
        line([-0.25 0.25], [0 0], 'Color', 'k', 'LineStyle', ':');
        yline(0.18, 'r--', '18cm Target');

        title(sprintf('PaTS-Wheel Kinematic Chain (T = %.2fs)', time(i)));
        pause(0.1);
    end
end