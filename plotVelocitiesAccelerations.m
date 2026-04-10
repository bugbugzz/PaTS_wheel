function plotVelocitiesAccelerations(dqSol, ddqSol, time, v_c_sol, v_p_sol, a_c_sol, a_p_sol)
%{
This function plots the velocities and accelerations for the 5 bodies
of the PaTS-Wheel mechanism AND the specific Claw/Pad tips in the same plots.
%}

    % --- 1. VELOCITIES FIGURE ---
    figure('Name', 'PaTS-Wheel: Velocities', 'NumberTitle', 'off');
    
    % X Velocities
    subplot(3,1,1); hold on; grid on;
    plot(time, dqSol(1,:), time, dqSol(4,:), time, dqSol(7,:), time, dqSol(10,:), time, dqSol(13,:), 'LineWidth', 1.5);
    plot(time, v_c_sol(1,:), 'g--', 'LineWidth', 2); % Claw Tip
    plot(time, v_p_sol(1,:), 'm--', 'LineWidth', 2); % Pad Tip
    title('X - Velocities (CM and Tips)'); ylabel('v_x [m/s]');
    legend('B1 (Coupler)', 'B2 (Claw Supp)', 'B3 (Claw Tip)', 'B4 (Pad Supp)', 'B5 (Pad Tip)', 'Claw Hook (c)', 'Pad Push (p)', 'Location', 'bestoutside');
    
    % Y Velocities
    subplot(3,1,2); hold on; grid on;
    plot(time, dqSol(2,:), time, dqSol(5,:), time, dqSol(8,:), time, dqSol(11,:), time, dqSol(14,:), 'LineWidth', 1.5);
    plot(time, v_c_sol(2,:), 'g--', 'LineWidth', 2); % Claw Tip
    plot(time, v_p_sol(2,:), 'm--', 'LineWidth', 2); % Pad Tip
    title('Y - Velocities (CM and Tips)'); ylabel('v_y [m/s]');
    
    % Angular Velocities (Tips share the angular velocity of their parent bodies)
    subplot(3,1,3); hold on; grid on;
    plot(time, dqSol(3,:), time, dqSol(6,:), time, dqSol(9,:), time, dqSol(12,:), time, dqSol(15,:), 'LineWidth', 1.5);
    title('Angular Velocities'); ylabel('\omega [rad/s]'); xlabel('Time [s]');


    % --- 2. ACCELERATIONS FIGURE ---
    figure('Name', 'PaTS-Wheel: Accelerations', 'NumberTitle', 'off');
    
    % X Accelerations
    subplot(3,1,1); hold on; grid on;
    plot(time, ddqSol(1,:), time, ddqSol(4,:), time, ddqSol(7,:), time, ddqSol(10,:), time, ddqSol(13,:), 'LineWidth', 1.5);
    plot(time, a_c_sol(1,:), 'g--', 'LineWidth', 2); % Claw Tip
    plot(time, a_p_sol(1,:), 'm--', 'LineWidth', 2); % Pad Tip
    title('X - Accelerations (CM and Tips)'); ylabel('a_x [m/s^2]');
    legend('B1 (Coupler)', 'B2 (Claw Supp)', 'B3 (Claw Tip)', 'B4 (Pad Supp)', 'B5 (Pad Tip)', 'Claw Hook (c)', 'Pad Push (p)', 'Location', 'bestoutside');
    
    % Y Accelerations
    subplot(3,1,2); hold on; grid on;
    plot(time, ddqSol(2,:), time, ddqSol(5,:), time, ddqSol(8,:), time, ddqSol(11,:), time, ddqSol(14,:), 'LineWidth', 1.5);
    plot(time, a_c_sol(2,:), 'g--', 'LineWidth', 2); % Claw Tip
    plot(time, a_p_sol(2,:), 'm--', 'LineWidth', 2); % Pad Tip
    title('Y - Accelerations (CM and Tips)'); ylabel('a_y [m/s^2]');
    
    % Angular Accelerations
    subplot(3,1,3); hold on; grid on;
    plot(time, ddqSol(3,:), time, ddqSol(6,:), time, ddqSol(9,:), time, ddqSol(12,:), time, ddqSol(15,:), 'LineWidth', 1.5);
    title('Angular Accelerations'); ylabel('\alpha [rad/s^2]'); xlabel('Time [s]');
end