clear all; close all; clc;

%% 1. SYSTEM PARAMETERS & CONFIGURATION
% -------------------------------------------------------------------------
% HINGE & BODY INDEX MAP (Used for h_flex, m, I, Qc, M, and Stress Output)
% Index 1: Coupler Hinge     (Connects Hub Link 1 to Claw Link 3)
% Index 2: Claw Base Hinge   (Connects Base Link 2 to Claw Link 3)
% Index 3: Claw Tip Body     (Solid geometry hook, does NOT bend)
% Index 4: Pad Base Hinge    (Connects Hub Link 1 to Pad Link 5)
% Index 5: Pad Tip Hinge     (Connects Base Link 4 to Pad Link 5)
% -------------------------------------------------------------------------
scale = 2.57;               % Multiplier to clear 18cm step (scales base 100mm design)
b_flex = 20 / 1000;         % flexure extrusion width (m)
L_notch = 8 / 1000;         % Bending zone length for virtual work (m)
D = 10 / 1000;              % Diameter of rigid link segments (m)
TPU_yield = 37.9e6;           % TPU 95A yield strength (Pa)
rho_tpu = 1200;             % TPU density (kg/m^3)
E_tpu = 67e6;               % TPU Young's Modulus (Pa)
t_start = 0;                % Simulation start time (s)
dt = 0.005;                 % Time step (s)
t_stop = 4;                 % Simulation end time (s)
freq = 0.5;                 % Motor driving frequency (Hz)
A_theta = deg2rad(20);      % Motor driving amplitude (rad)
th1_0 = 0.7;                % Hub/Motor initial start angle (rad)
th2_0 = 2.0;                % Claw support initial start angle (rad)
th4_0 = 1.0;                % Pad support initial start angle (rad)
% Array of Flexure Thicknesses [Indices 1 to 5] (m)
h_flex = [1.5; 1; 3.0; 1; 1.] / 1000;% Array of Link Lengths (m)
% L_vals mapping based on geometric constraints:
% L1: Hub-to-Claw-Pivot offset (m)
% L2: Hub-to-Pad-Joint radius (m)
% L3: Hub-to-Pad-Pivot offset (m)
% L4: Hub-to-Claw-Joint radius (m)
% L5: Claw Support length (m)
% L6: Tip Body length (Claw/Pad) (m)
% L7: Unused in constraints (m)
% L8: Pad Support length (m)
L_vals = ([50; 25; 50; 25; 25; 25; 25; 25] * scale) / 1000; 

%% 2. INITIALIZATION & SYMBOLIC SETUP
% Calculate mass and inertia for the 5 moving bodies
m = rho_tpu * L_vals(1:5) * pi * (D/2)^2; 
I = (1/12) * m .* L_vals(1:5).^2;         
% Build 15x15 diagonal mass matrix for Lagrange multipliers
M = zeros(15);
for i = 1:5
    ind = 3*(i-1)+1:3*i; 
    M(ind,ind) = diag([m(i), m(i), I(i)]);
end
% Define symbolic variables for the math engine
q = sym('q',[15 1]);   
dq = sym('dq',[15 1]); 
syms t theta_driver dtheta_driver ddtheta_driver 
% Unpack state vectors into readable coordinates
x1=q(1); y1=q(2); th1=q(3); 
x2=q(4); y2=q(5); th2=q(6);
x3=q(7); y3=q(8); th3=q(9); 
x4=q(10);y4=q(11);th4=q(12); 
x5=q(13);y5=q(14);th5=q(15);
% Define tracking coordinates for the Claw (c) and Pad (p) tips
b3_sym = [x1 + L_vals(4)*cos(th1 + pi - 1.4); y1 + L_vals(4)*sin(th1 + pi - 1.4)];
a2_sym = [x1 + L_vals(2)*cos(th1); y1 + L_vals(2)*sin(th1)];
tip_height = L_vals(6); 
c_sym = [b3_sym(1) - tip_height*sin(th3); b3_sym(2) + tip_height*cos(th3)]; 
p_sym = [a2_sym(1) - tip_height*sin(th5); a2_sym(2) + tip_height*cos(th5)];
% Convert tip coordinates to fast numerical Jacobian functions
Jp_f = matlabFunction(jacobian(p_sym, q), 'vars', {q});
Jc_f = matlabFunction(jacobian(c_sym, q), 'vars', {q});
% Define the geometric rules (Constraints) that hold the 1-DOF wheel together
C = [x1; y1; ... 
     x2 - 0.5*L_vals(5)*cos(th2) - (-L_vals(1)); y2 - 0.5*L_vals(5)*sin(th2); ... 
     x4 - 0.5*L_vals(8)*cos(th4) - L_vals(3); y4 - 0.5*L_vals(8)*sin(th4); ...    
     (x1 + L_vals(4)*cos(th1 + pi - 1.4)) - (x3 + 0.5*L_vals(6)*cos(th3)); ...    
     (y1 + L_vals(4)*sin(th1 + pi - 1.4)) - (y3 + 0.5*L_vals(6)*sin(th3));
     (x2 + 0.5*L_vals(5)*cos(th2)) - (x3 - 0.5*L_vals(6)*cos(th3)); ...           
     (y2 + 0.5*L_vals(5)*sin(th2)) - (y3 - 0.5*L_vals(6)*sin(th3));
     (x1 + L_vals(2)*cos(th1)) - (x5 - 0.5*L_vals(6)*cos(th5)); ...               
     (y1 + L_vals(2)*sin(th1)) - (y5 - 0.5*L_vals(6)*sin(th5));
     (x4 + 0.5*L_vals(8)*cos(th4)) - (x5 + 0.5*L_vals(6)*cos(th5)); ...           
     (y4 + 0.5*L_vals(8)*sin(th4)) - (y5 + 0.5*L_vals(6)*sin(th5));
     th1 - (th1_0 + theta_driver)];                                               
% Convert symbolic math constraints into numerical functions for the solver
C_f = matlabFunction(C, 'vars', {q, t, theta_driver}); 
Cq_f = matlabFunction(jacobian(C,q), 'vars', {q, t, theta_driver}); 
Ct_f = matlabFunction([zeros(14,1); -dtheta_driver], 'vars', {q, t, theta_driver, dtheta_driver}); 
Ctt_f = matlabFunction([zeros(14,1); -ddtheta_driver], 'vars', {q, t, theta_driver, dtheta_driver, ddtheta_driver}); 
Cqt_f = @(q,t,th,dth,ddth) zeros(15,15); 
Cqqdq_f = matlabFunction(jacobian(jacobian(C,q)*dq,q), 'vars', {q, dq, t, theta_driver}); 

%% 3. SIMULATION LOOP PREPARATION
time = t_start:dt:t_stop;
qSol = NaN(15,length(time)); 
dqSol = qSol; 
ddqSol = qSol; 
Qc = NaN(15,length(time));
v_c_sol = NaN(2, length(time)); 
v_p_sol = NaN(2, length(time)); 
% Provide the Newton-Raphson solver with geometric starting guesses
q0 = zeros(15,1); 
q0(1:3) = [0; 0; th1_0]; 
q0(4:6) = [-L_vals(1) + 0.5*L_vals(5)*cos(th2_0); 0.5*L_vals(5)*sin(th2_0); th2_0]; 
q0(10:12) = [L_vals(3) + 0.5*L_vals(8)*cos(th4_0); 0.5*L_vals(8)*sin(th4_0); th4_0];

%% 4. EXECUTE KINEMATIC & DYNAMIC SIMULATION
for i = 1:length(time)
    % Calculate motor sine-wave inputs
    th_val   = A_theta * sin(2 * pi * freq * time(i)); 
    dth_val  = A_theta * 2 * pi * freq * cos(2 * pi * freq * time(i));
    ddth_val = -A_theta * (2 * pi * freq)^2 * sin(2 * pi * freq * time(i));
    
    % Inject current timestep variables into function wrappers
    C_step     = @(qq, tt) C_f(qq, tt, th_val);
    Cq_step    = @(qq, tt) Cq_f(qq, tt, th_val);
    Ct_step    = @(qq, tt) Ct_f(qq, tt, th_val, dth_val);
    Cqt_step   = @(qq, tt) Cqt_f(qq, tt, th_val, dth_val, ddth_val);
    Ctt_step   = @(qq, tt) Ctt_f(qq, tt, th_val, dth_val, ddth_val);
    Cqqdq_step = @(qq, dqq) Cqqdq_f(qq, dqq, time(i), th_val); 
    % Solve exact positions using Newton-Raphson
    if i == 1
        qSol(:,1) = nietraphson(q0, 0, C_step, Cq_step, 1e-7, 1e-7, 150); 
    else
        qSol(:,i) = nietraphson(qSol(:,i-1), time(i), C_step, Cq_step, 1e-7, 1e-7, 150); 
    end
    
    % Solve velocities and accelerations
    dqSol(:,i) = velocities(qSol(:,i), time(i), Cq_step, Ct_step);
    ddqSol(:,i) = accelerations(qSol(:,i), dqSol(:,i), time(i), Cq_step, Cqt_step, Ctt_step, Cqqdq_step);
    % Solve internal linkage reaction forces
    lambda = lagrangeMultipliers(qSol(:,i), ddqSol(:,i), time(i), Cq_step, M); 
    Qc(:,i) = -Cq_step(qSol(:,i), time(i))' * lambda; 
    
    % Record spatial tracking
    v_p_sol(:,i) = Jp_f(qSol(:,i)) * dqSol(:,i);
    v_c_sol(:,i) = Jc_f(qSol(:,i)) * dqSol(:,i);
end

%% 5. STRESS ANALYSIS CALCULATION
% Numerical differentiation for tip accelerations
a_c_sol = [diff(v_c_sol, 1, 2) / dt, [0;0]]; 
a_p_sol = [diff(v_p_sol, 1, 2) / dt, [0;0]];
% Run PRBM analysis and fetch safety factors
[~, s_axial, s_bend, SF] = calculateLinkStresses(Qc, b_flex, h_flex, TPU_yield, L_vals);

%% 6. ACTIVATION FORCE CALCULATION (Virtual Work)
disp('Calculating Required Pad Activation Force...');
% Pre-calculate torsional stiffness (k) for all active flexures [Nm/rad]
I_flex_array = (b_flex .* h_flex.^3) / 12; 
k_spring = (E_tpu .* I_flex_array) / L_notch;
F_pad_required = zeros(1, length(time));
Total_Hinge_Torque = zeros(1, length(time));
for i = 1:length(time)
    q = qSol(:, i);
    dq = dqSol(:, i);
    
    % Unpack absolute angles and velocities
    th1 = q(3); th2 = q(6); th3 = q(9); th4 = q(12); th5 = q(15);
    dth1 = dq(3); dth2 = dq(6); dth3 = dq(9); dth4 = dq(12); dth5 = dq(15);
    
    % Define the unstrained neutral state from t=0
    th1_0 = qSol(3,1); th2_0 = qSol(6,1); th3_0 = qSol(9,1); 
    th4_0 = qSol(12,1); th5_0 = qSol(15,1);
    
    % Calculate geometric bending (Delta Theta) at each hinge connection
    bend_13 = abs((th3 - th1) - (th3_0 - th1_0)); 
    bend_23 = abs((th3 - th2) - (th3_0 - th2_0)); 
    bend_15 = abs((th5 - th1) - (th5_0 - th1_0)); 
    bend_45 = abs((th5 - th4) - (th5_0 - th4_0)); 
    
    % Calculate resisting torques using correct mapped array indices
    % Note: Index 3 is skipped as it represents the solid Claw Tip geometry
    tau_13 = k_spring(1) * bend_13; % Index 1: Coupler Hinge
    tau_23 = k_spring(2) * bend_23; % Index 2: Claw Base Hinge
    tau_15 = k_spring(4) * bend_15; % Index 4: Pad Base Hinge
    tau_45 = k_spring(5) * bend_45; % Index 5: Pad Tip Hinge
    
    % Calculate instantaneous angular velocities of the bends
    w_13 = abs(dth3 - dth1);
    w_23 = abs(dth3 - dth2);
    w_15 = abs(dth5 - dth1);
    w_45 = abs(dth5 - dth4);
    
    % Balance internal power against horizontal pad displacement
    Power_internal = (tau_13 * w_13) + (tau_23 * w_23) + (tau_15 * w_15) + (tau_45 * w_45);
    v_pad_x = abs(v_p_sol(1, i)); 
    
    if v_pad_x > 1e-4 
        F_pad_required(i) = Power_internal / v_pad_x;
    else
        F_pad_required(i) = 0; % Mitigate division-by-zero math artifacts
    end
end

%% 7. OUTPUTS, REPORTS & FIGURES
% --- 7.1 Console Reports ---
fprintf('\n--- STRESS REPORT: 1.3kg Robot Climbing 18cm Step ---\n');
fprintf('%-25s | %-12s | %-12s | %-8s\n', 'Flexure Node', 'Axial (MPa)', 'Bending (MPa)', 'SF');
fprintf('----------------------------------------------------------------------\n');
% Array of physical names mapped to Indices 1 through 5 for the console report
node_names = {'Coupler Hinge (Index 1)','Claw Base Hinge (Index 2)','Claw Tip Hook (Index 3)','Pad Base Hinge (Index 4)', 'Pad Tip Hinge (Index 5)'};
for i = 1:5
    fprintf('%-25s | %-12.4f | %-12.4f | %-8.2f\n', ...
            node_names{i}, max(s_axial(i,:))/1e6, max(s_bend(i,:))/1e6, SF(i));
end

printSystemParameters(L_vals, b_flex, h_flex, scale);
disp('Simulation Complete.');

% --- 7.2 Plotting & Visualization ---
plotConfiguration(qSol, time, L_vals, 20);
% plotVelocitiesAccelerations(dqSol, ddqSol, time, v_c_sol, v_p_sol, a_c_sol, a_p_sol);
% plotReactionForces(Qc, time); 

figure('Name', 'Required Activation Force', 'Color', 'w');
plot(time, F_pad_required, 'LineWidth', 3, 'Color', [0.85 0.325 0.098]);
grid on;
title('Force Required at Pad to Unfold Wheel');
xlabel('Time [s]');
ylabel('Push Force [Newtons]');
yline(12.75, 'r--', 'Robot Weight (1.3kg)');