function plotVelocitiesAccelerations(dqSol, ddqSol, time, v_c_sol, v_p_sol)
%{
This function plots the velocities and accelerations for the 5 bodies
of the PaTS-Wheel mechanism, AND the specific Claw/Pad tips.
%}

    % --- 1. CM VELOCITIES FIGURE ---
    figure('Name', 'PaTS-Wheel: CM Velocities', 'NumberTitle', 'off');
    subplot(2,1,1); hold on; grid on;
    plot(time, dqSol(1,:), time, dqSol(4,:), time, dqSol(7,:), time, dqSol(10,:), time, dqSol(13,:), 'LineWidth', 1.5);
    title('Center of Mass X - Velocities'); ylabel('v_x [m/s]');
    legend('Coupler (B1)', 'Claw Support (B2)', 'Claw Tip (B3)', 'Pad Support (B4)', 'Pad Tip (B5)', 'Location', 'bestoutside');
    
    subplot(2,1,2); hold on; grid on;
    plot(time, dqSol(2,:), time, dqSol(5,:), time, dqSol(8,:), time, dqSol(11,:), time, dqSol(14,:), 'LineWidth', 1.5);
    title('Center of Mass Y - Velocities'); ylabel('v_y [m/s]'); xlabel('Time [s]');

    % --- 2. CM ACCELERATIONS FIGURE ---
    figure('Name', 'PaTS-Wheel: CM Accelerations', 'NumberTitle', 'off');
    subplot(2,1,1); hold on; grid on;
    plot(time, ddqSol(1,:), time, ddqSol(4,:), time, ddqSol(7,:), time, ddqSol(10,:), time, ddqSol(13,:), 'LineWidth', 1.5);
    title('Center of Mass X - Accelerations'); ylabel('a_x [m/s^2]');
    legend('Coupler (B1)', 'Claw Support (B2)', 'Claw Tip (B3)', 'Pad Support (B4)', 'Pad Tip (B5)', 'Location', 'bestoutside');
    
    subplot(2,1,2); hold on; grid on;
    plot(time, ddqSol(2,:), time, ddqSol(5,:), time, ddqSol(8,:), time, ddqSol(11,:), time, ddqSol(14,:), 'LineWidth', 1.5);
    title('Center of Mass Y - Accelerations'); ylabel('a_y [m/s^2]'); xlabel('Time [s]');

    % --- 3. NEW: TIP KINEMATICS FIGURE ---
    figure('Name', 'PaTS-Wheel: Hook Tips (c and p)', 'NumberTitle', 'off');
    
    % X Velocities of the Tips
    subplot(2,1,1); hold on; grid on;
    plot(time, v_c_sol(1,:), 'g-', 'LineWidth', 2);
    plot(time, v_p_sol(1,:), 'm-', 'LineWidth', 2);
    title('Tip X-Velocities (Horizontal Reach)'); 
    ylabel('Velocity [m/s]');
    legend('Claw Tip (c)', 'Pad Tip (p)', 'Location', 'best');
    
    % Y Velocities of the Tips
    subplot(2,1,2); hold on; grid on;
    plot(time, v_c_sol(2,:), 'g--', 'LineWidth', 2);
    plot(time, v_p_sol(2,:), 'm--', 'LineWidth', 2);
    title('Tip Y-Velocities (Vertical Lift)'); 
    ylabel('Velocity [m/s]'); xlabel('Time [s]');
end