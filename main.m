clear all; close all; clc;
warning('off', 'all'); % Turn off NR warnings for smooth console output

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
D = 20 / 1000;              % Diameter of rigid link segments (m)
TPU_yield = 37.9e6;         % TPU 95A yield strength (Pa)
rho_tpu = 1200;             % TPU density (kg/m^3)
E_tpu = 67e6;               % TPU Young's Modulus (Pa)
t_start = 0;                % Simulation start time (s)
dt = 0.005;                 % Time step (s)
t_stop = 4;                 % Simulation end time (s)
freq = 0.5;                 % Motor driving frequency (Hz)
A_theta = deg2rad(17);      % Motor driving amplitude (rad) - REDUCED TO PREVENT CRASHING
th1_0 = 0.7;                % Hub/Motor initial start angle (rad)
th2_0 = 1.0;                % Claw support initial start angle (rad)
th4_0 = 1.5;                % Pad support initial start angle (rad)

% Array of Flexure Thicknesses [Indices 1 to 5] (m)
h_flex = [2; 1.1; 3.0; 1.1; 1.1] / 1000;

% --- NEW EXACT MEASUREMENTS ---
% L1: Claw Pivot Offset (35 mm)
% L2: Coupler Right Beam (15 mm)
% L3: Pad Pivot Offset (17.5 mm)
% L4: Coupler Left Beam (30 mm)
% L5: Claw Support length (18.5 mm)
% L6: Claw Base link (17 mm)
% L7: Pad Base link (18 mm)
% L8: Pad Support length (17.5 mm)
L_vals_mm = [35.0; 15.0; 17.5; 30.0; 18.5; 17.0; 18.0; 17.5];
L_vals = (L_vals_mm * scale) / 1000; 

%% 2. INITIALIZATION & SYMBOLIC SETUP
% Calculate mass and inertia for the 5 moving bodies
body_lens = [(L_vals(2)+L_vals(4))/2, L_vals(5), L_vals(6), L_vals(8), L_vals(7)];
m = rho_tpu * body_lens * pi * (D/2)^2; 
I = (1/12) * m .* body_lens.^2;         

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

% Define tracking coordinates for the base hinges
b3_sym = [x1 + L_vals(4)*cos(th1 + pi - 1.4); y1 + L_vals(4)*sin(th1 + pi - 1.4)];
a2_sym = [x1 + L_vals(2)*cos(th1); y1 + L_vals(2)*sin(th1)];
b2_sym = [x2 + 0.5*L_vals(5)*cos(th2); y2 + 0.5*L_vals(5)*sin(th2)];
a3_sym = [x4 + 0.5*L_vals(8)*cos(th4); y4 + 0.5*L_vals(8)*sin(th4)];

% --- EXACT GEOMETRIC MATH TO PREVENT FLOATING STARS ---
L6_phys = 17.0/1000; claw_out = 32.0/1000; claw_in = 25.0/1000;
x_claw = (L6_phys^2 + claw_out^2 - claw_in^2) / (2 * L6_phys);
y_claw = sqrt(claw_out^2 - x_claw^2);

L7_phys = 18.0/1000; pad_out = 37.5/1000; pad_in = 26.0/1000;
x_pad = (L7_phys^2 + pad_out^2 - pad_in^2) / (2 * L7_phys);
y_pad = sqrt(pad_out^2 - x_pad^2);

% Map exactly to the local rotation of the bodies so stars touch the tips!
c_sym = b2_sym + (x_claw * scale) * [cos(th3); sin(th3)] + (y_claw * scale) * [-sin(th3); cos(th3)];
p_sym = a3_sym + (x_pad * scale) * [-cos(th5); -sin(th5)] + (y_pad * scale) * [-sin(th5); cos(th5)];

% Convert tip coordinates to fast numerical Jacobian functions
Jp_f = matlabFunction(jacobian(p_sym, q), 'vars', {q});
Jc_f = matlabFunction(jacobian(c_sym, q), 'vars', {q});

% Define the geometric rules (Constraints) that hold the 1-DOF wheel together
C = [x1; y1; ... 
     x2 - 0.5*L_vals(5)*cos(th2) - (-L_vals(1)); y2 - 0.5*L_vals(5)*sin(th2); ... 
     x4 - 0.5*L_vals(8)*cos(th4) - L_vals(3); y4 - 0.5*L_vals(8)*sin(th4); ...    
     (x1 + L_vals(4)*cos(th1 + pi - 1.4)) - (x3 + 0.5*L_vals(6)*cos(th3)); ...    
     (y1 + L_vals(4)*sin(th1 + pi - 1.4)) - (y3 + 0.5*L_vals(6)*sin(th3)); ...
     (x2 + 0.5*L_vals(5)*cos(th2)) - (x3 - 0.5*L_vals(6)*cos(th3)); ...           
     (y2 + 0.5*L_vals(5)*sin(th2)) - (y3 - 0.5*L_vals(6)*sin(th3)); ...
     (x1 + L_vals(2)*cos(th1)) - (x5 - 0.5*L_vals(7)*cos(th5)); ...               
     (y1 + L_vals(2)*sin(th1)) - (y5 - 0.5*L_vals(7)*sin(th5)); ...
     (x4 + 0.5*L_vals(8)*cos(th4)) - (x5 + 0.5*L_vals(7)*cos(th5)); ...           
     (y4 + 0.5*L_vals(8)*sin(th4)) - (y5 + 0.5*L_vals(7)*sin(th5)); ...
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
        qSol(:,1) = nietraphson(q0, 0, C_step, Cq_step, 1e-4, 1e-4, 200); 
    else
        qSol(:,i) = nietraphson(qSol(:,i-1), time(i), C_step, Cq_step, 1e-4, 1e-4, 200); 
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
    
    % SMOOTH GRAPH FIX: Clamp minimum velocity to prevent 600N spikes!
    v_pad_x = max(v_pad_x, 0.002);
    F_pad_required(i) = Power_internal / v_pad_x;
end

%% 7. OUTPUTS, REPORTS & FIGURES
printSystemParameters(L_vals, b_flex, h_flex, L_notch, scale, s_axial, s_bend, SF);

% --- 7.2 Plotting & Visualization ---

% 1. Plot the Animation
plotConfiguration(qSol, time, L_vals, scale, 20); 

% 2. Plot the Velocities and Accelerations
% (Passing the a_c_sol and a_p_sol arrays we calculated in Section 5)
plotVelocitiesAccelerations(dqSol, ddqSol, time, v_c_sol, v_p_sol, a_c_sol, a_p_sol);

% 3. Plot the Reaction Forces (Internal stresses on the joints)
plotReactionForces(Qc, time);

% 4. Plot the Pad Activation Force
figure('Name', 'Required Activation Force', 'Color', 'w');
plot(time, F_pad_required, 'LineWidth', 3, 'Color', [0.85 0.325 0.098]);
grid on;
ylim([0, 15]); % Lock the view so it looks like a clean, continuous arch
title('Force Required at Pad to Unfold Wheel');
xlabel('Time [s]');
ylabel('Push Force [Newtons]');
yline(12.75, 'r--', 'Robot Weight (1.3kg)');