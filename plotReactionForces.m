function plotReactionForces(Qc, time)
%{
This function plots the generalized reaction forces (Fx, Fy, Moment) 
acting on the center of mass of each of the 5 TPU bodies.
%}

    figure('Name', 'PaTS-Wheel: Internal Body Forces', 'NumberTitle', 'off');
    
    % --- 1. Forces in X (Indices 1, 4, 7, 10, 13) ---
    subplot(3,1,1); hold on; grid on;
    plot(time, Qc(1,:), 'LineWidth', 1.5);
    plot(time, Qc(4,:), 'LineWidth', 1.5);
    plot(time, Qc(7,:), 'LineWidth', 1.5);
    plot(time, Qc(10,:), 'LineWidth', 1.5);
    plot(time, Qc(13,:), 'LineWidth', 1.5);
    title('Internal Reaction Forces in X-Direction'); 
    ylabel('F_x [N]');
    legend('Coupler (B1)', 'Claw Support (B2)', 'Claw Tip (B3)', 'Pad Support (B4)', 'Pad Tip (B5)', 'Location', 'bestoutside');
    
    % --- 2. Forces in Y (Indices 2, 5, 8, 11, 14) ---
    subplot(3,1,2); hold on; grid on;
    plot(time, Qc(2,:), 'LineWidth', 1.5);
    plot(time, Qc(5,:), 'LineWidth', 1.5);
    plot(time, Qc(8,:), 'LineWidth', 1.5);
    plot(time, Qc(11,:), 'LineWidth', 1.5);
    plot(time, Qc(14,:), 'LineWidth', 1.5);
    title('Internal Reaction Forces in Y-Direction'); 
    ylabel('F_y [N]');
    
    % --- 3. Reaction Moments / Bending (Indices 3, 6, 9, 12, 15) ---
    subplot(3,1,3); hold on; grid on;
    plot(time, Qc(3,:), 'LineWidth', 1.5);
    plot(time, Qc(6,:), 'LineWidth', 1.5);
    plot(time, Qc(9,:), 'LineWidth', 1.5);
    plot(time, Qc(12,:), 'LineWidth', 1.5);
    plot(time, Qc(15,:), 'LineWidth', 1.5);
    title('Internal Reaction Moments (Bending)'); 
    ylabel('Moment [Nm]'); 
    xlabel('Time [s]');
end