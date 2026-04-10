function plotConfiguration(qSol, time, L_vals, plot_stepsize)
%{
This function animates the 5-body PaTS-Wheel mechanism over time.
INPUT:
    qSol          Matrix of generalized coordinates (15 x TimeSteps)
    time          Array of time values
    L_vals        Link lengths
    plot_stepsize How many time steps to skip between frames (for speed)
%}
    figure('Name', 'PaTS-Wheel Mechanism Animation', 'NumberTitle', 'off');
    set(gcf, 'Position', [100, 100, 800, 500]); % Make window nice and wide
    
    % Loop through the solution over time
    for i = 1:plot_stepsize:length(time)
        % 1. Extract coordinates for this time step
        q = qSol(:, i);
        x1=q(1); y1=q(2); th1=q(3);    % Body 1: Coupler
        x2=q(4); y2=q(5); th2=q(6);    % Body 2: Claw Support
        x3=q(7); y3=q(8); th3=q(9);    % Body 3: Claw Tip
        x4=q(10);y4=q(11);th4=q(12);   % Body 4: Pad Support
        x5=q(13);y5=q(14);th5=q(15);   % Body 5: Pad Tip

        % 2. Calculate Joint Positions
        % Fixed Ground Points
        O  = [0, 0];                        % Origin (b4/a1)
        b1 = [-L_vals(1), 0];               % Claw Ground Pin
        a4 = [L_vals(3), 0];                % Pad Ground Pin

        % Body 1 Joints (Central Coupler) - TRUE RIGID V-SHAPE
        b3 = [x1 + L_vals(4)*cos(th1 + pi - 1.4), y1 + L_vals(4)*sin(th1 + pi - 1.4)];
        a2 = [x1 + L_vals(2)*cos(th1), y1 + L_vals(2)*sin(th1)];

        % Body 2 Joint (Claw Support Tip)
        b2 = [x2 + 0.5*L_vals(5)*cos(th2), y2 + 0.5*L_vals(5)*sin(th2)];

        % Body 4 Joint (Pad Support Tip)
        a3 = [x4 + 0.5*L_vals(8)*cos(th4), y4 + 0.5*L_vals(8)*sin(th4)];

        % Calculate aesthetic tips for the Claw (c) and Pad (p)
        % Using true +90 degree perpendicular projection to force them UP and INWARD
        tip_height = L_vals(6); 
        
        % Claw tip (c) anchored to b3, pointing UP (+) and INWARD (-)
        c = [b3(1) - tip_height*sin(th3), b3(2) + tip_height*cos(th3)]; 
        
        % Pad tip (p) anchored to a2, pointing UP (+) and INWARD (-)
        p = [a2(1) - tip_height*sin(th5), a2(2) + tip_height*cos(th5)];

        % 3. Draw the Mechanism
        clf; % Clear previous frame
        hold on;
        grid on;
        axis equal;
        
        % Set plot limits 
        axis([-0.20 0.20 -0.05 0.25]);
        title(sprintf('PaTS-Wheel Simulation | Time = %.2f s', time(i)));
        xlabel('X Position [m]'); ylabel('Y Position [m]');

        % --- Plot Links as Polygons/Lines ---
        
        % Body 1: Central Inverting Coupler (Triangle: b3 - O - a2)
        plot([b3(1), O(1), a2(1), b3(1)], [b3(2), O(2), a2(2), b3(2)], 'k-', 'LineWidth', 2.5);
        fill([b3(1), O(1), a2(1)], [b3(2), O(2), a2(2)], [0.8 0.8 0.8], 'FaceAlpha', 0.5); % Gray fill

        % Body 2: Claw Support (Line: b1 - b2)
        plot([b1(1), b2(1)], [b1(2), b2(2)], 'b-', 'LineWidth', 2);

        % Body 4: Pad Support (Line: a4 - a3)
        plot([a4(1), a3(1)], [a4(2), a3(2)], 'r-', 'LineWidth', 2);

        % Body 3: Claw Tip (Triangle: b2 - c - b3)
        plot([b2(1), c(1), b3(1), b2(1)], [b2(2), c(2), b3(2), b2(2)], 'b-', 'LineWidth', 2);
        fill([b2(1), c(1), b3(1)], [b2(2), c(2), b3(2)], 'b', 'FaceAlpha', 0.2);

        % Body 5: Pad Tip (Triangle: a2 - p - a3)
        plot([a2(1), p(1), a3(1), a2(1)], [a2(2), p(2), a3(2), a2(2)], 'r-', 'LineWidth', 2);
        fill([a2(1), p(1), a3(1)], [a2(2), p(2), a3(2)], 'r', 'FaceAlpha', 0.2);

        % --- Plot Joints (Pins) ---
        joints_x = [O(1), b1(1), a4(1), b3(1), a2(1), b2(1), a3(1)];
        joints_y = [O(2), b1(2), a4(2), b3(2), a2(2), b2(2), a3(2)];
        plot(joints_x, joints_y, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5); % Black dots for pins

        % Ground Base Line (Visual reference)
        plot([-0.08, 0.08], [0, 0], 'k--', 'LineWidth', 1);

        % --- Plot Specific Tip Markers ---
        % Green star for the Claw Tip (The Hook)
        plot(c(1), c(2), 'g*', 'MarkerSize', 10, 'LineWidth', 1.5); 
        % Magenta star for the Pad Tip (The Push Point)
        plot(p(1), p(2), 'm*', 'MarkerSize', 10, 'LineWidth', 1.5); 

        drawnow;
    end
end