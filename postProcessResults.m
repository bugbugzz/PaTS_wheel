function postProcessResults(time, F_claw_output, lambdaSol, qSol, dqSol, Qc)
    % 1. Extract Driver Torque (Constraint 15)
    % lambda(15) is the force/torque associated with the driver constraint
    T_coupler = -lambdaSol(15, :); 

    % 2. Plot the Claw Grip Force
    figure('Name', 'PaTS-Wheel: Grip Force Transmission', 'NumberTitle', 'off');
    plot(time, F_claw_output, 'g-', 'LineWidth', 2);
    grid on;
    title('Claw Grip Force (Assuming 10N Input on Pad)');
    xlabel('Time [s]');
    ylabel('Claw Output Force [N]');

    % 3. Plot the Driver Torque (Optional but recommended)
    figure('Name', 'PaTS-Wheel: Driver Torque', 'NumberTitle', 'off');
    plot(time, T_coupler, 'b-', 'LineWidth', 2);
    grid on;
    title('Torque Required to Drive the Coupler');
    xlabel('Time [s]');
    ylabel('Torque [Nm]');

    % 4. Save results to .mat file
    save('PaTS_Wheel_Forces.mat', 'T_coupler', 'time', 'qSol', 'dqSol', 'lambdaSol', 'Qc', 'F_claw_output');
    
    disp('Post-processing complete. Results saved to PaTS_Wheel_Forces.mat.');
end