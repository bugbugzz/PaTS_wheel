% Run this after main.m to use the variables qSol and L_vals
figure('Name', 'PaTS-Wheel: The 5 Moving Bodies Only', 'Color', 'w');
hold on; axis equal; grid on;

% Set the frame to visualize (e.g., the first frame)
frame = 1;
q = qSol(:, frame);
L = L_vals;

% Define distinct colors for each body
body_colors = [1 0 0; 0 0.7 0; 0 0 1; 1 0.6 0; 0.7 0 0.7]; % Red, Green, Blue, Orange, Purple

for i = 1:5
    % Extract coordinates for Body i (x, y, theta)
    idx = 3*(i-1)+1;
    x = q(idx);
    y = q(idx+1);
    th = q(idx+2);
    
    % Determine half-lengths for drawing from the center
    half_L = L(i)/2;
    
    % Calculate start and end points of the rigid link
    x_coords = [x - half_L*cos(th), x + half_L*cos(th)];
    y_coords = [y - half_L*sin(th), y + half_L*sin(th)];
    
    % Plot the specific link
    plot(x_coords, y_coords, 'LineWidth', 6, 'Color', body_colors(i,:));
    
    % Label the body center (the CoM tracked by the simulation)
    plot(x, y, 'ko', 'MarkerFaceColor', 'k'); 
    text(x, y + 0.02, sprintf('Body %d', i), 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
end

% Set the view to show the entire 257mm mechanism
xlim([-0.2, 0.2]); ylim([-0.2, 0.2]);
title('Visualizing the 5 Moving Bodies (PRBM Simulation Links)');
xlabel('x [m]'); ylabel('y [m]');