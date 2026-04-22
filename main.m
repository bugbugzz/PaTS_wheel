clear all; close all; clc;
warning('off', 'all'); % turn off the solver warnings so the console stays clean

%% Setup system parameters
% quick map of what each index means for the matrices:
% index 1: coupler hinge     (hub link 1 to claw link 3)
% index 2: claw base hinge   (base link 2 to claw link 3)
% index 3: claw tip body     (the solid hook, doesn't bend)
% index 4: pad base hinge    (hub link 1 to pad link 5)
% index 5: pad tip hinge     (base link 4 to pad link 5)

scale = 2.57;               % scale up to clear the 18cm step
b_flex = 20 / 1000;         % extrusion width of the wheel
L_notch = 8 / 1000;         % active bending length of the leaf spring
D = 20 / 1000;              % diameter of the rigid parts
TPU_yield = 37.9e6;         % yield strength for TPU 95A
rho_tpu = 1200;             % density of TPU
E_tpu = 67e6;               % youngs modulus for TPU

t_start = 0;                
dt = 0.005;                 
t_stop = 4;                 
freq = 0.5;                 % motor driving frequency
A_theta = deg2rad(17);      % motor swing amplitude

th1_0 = 0.7;                % start angle for the hub
th2_0 = 1.0;                % start angle for claw support
th4_0 = 1.5;                % start angle for pad support

% OPTIMIZED: Flexure minimum thicknesses (waist) for each hinge
h_flex = [2.0; 1.5; 3.0; 2.0; 2.0] / 1000;

% OPTIMIZED: Custom JPE Notch Diameters 
D_flex = [8.0; 14.0; 12.0; 8.0; 8.0] / 1000; 

% real physical measurements of the 3d print (in mm)
L_vals_mm = [35.0; 15.0; 17.5; 30.0; 18.5; 17.0; 18.0; 17.5];
L_vals = (L_vals_mm * scale) / 1000; 

%% Setup symbolic math and physics
% calc mass and inertia for the 5 main moving parts
body_lens = [(L_vals(2)+L_vals(4))/2, L_vals(5), L_vals(6), L_vals(8), L_vals(7)];
m = rho_tpu * body_lens * pi * (D/2)^2; 
I = (1/12) * m .* body_lens.^2;         

% build the 15x15 mass matrix for lagrange multipliers
M = zeros(15);
for i = 1:5
    ind = 3*(i-1)+1:3*i; 
    M(ind,ind) = diag([m(i), m(i), I(i)]);
end

% setup symbolic variables for the solver
q = sym('q',[15 1]);   
dq = sym('dq',[15 1]); 
syms t theta_driver dtheta_driver ddtheta_driver 

x1=q(1); y1=q(2); th1=q(3); 
x2=q(4); y2=q(5); th2=q(6);
x3=q(7); y3=q(8); th3=q(9); 
x4=q(10);y4=q(11);th4=q(12); 
x5=q(13);y5=q(14);th5=q(15);

% coordinate tracking for the hinges
b3_sym = [x1 + L_vals(4)*cos(th1 + pi - 1.4); y1 + L_vals(4)*sin(th1 + pi - 1.4)];
a2_sym = [x1 + L_vals(2)*cos(th1); y1 + L_vals(2)*sin(th1)];
b2_sym = [x2 + 0.5*L_vals(5)*cos(th2); y2 + 0.5*L_vals(5)*sin(th2)];
a3_sym = [x4 + 0.5*L_vals(8)*cos(th4); y4 + 0.5*L_vals(8)*sin(th4)];

% use law of cosines to find the exact geometric tips of the solid bodies
L6_phys = 17.0/1000; claw_out = 32.0/1000; claw_in = 25.0/1000;
x_claw = (L6_phys^2 + claw_out^2 - claw_in^2) / (2 * L6_phys);
y_claw = sqrt(claw_out^2 - x_claw^2);

L7_phys = 18.0/1000; pad_out = 37.5/1000; pad_in = 26.0/1000;
x_pad = (L7_phys^2 + pad_out^2 - pad_in^2) / (2 * L7_phys);
y_pad = sqrt(pad_out^2 - x_pad^2);

% glue the tips to the local rotation of the bodies
c_sym = b2_sym + (x_claw * scale) * [cos(th3); sin(th3)] + (y_claw * scale) * [-sin(th3); cos(th3)];
p_sym = a3_sym + (x_pad * scale) * [-cos(th5); -sin(th5)] + (y_pad * scale) * [-sin(th5); cos(th5)];

% convert to matlab functions so the solver runs fast
Jp_f = matlabFunction(jacobian(p_sym, q), 'vars', {q});
Jc_f = matlabFunction(jacobian(c_sym, q), 'vars', {q});

% the kinematic rules that hold the wheel together
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

% prep the matrices for newton raphson
C_f = matlabFunction(C, 'vars', {q, t, theta_driver}); 
Cq_f = matlabFunction(jacobian(C,q), 'vars', {q, t, theta_driver}); 
Ct_f = matlabFunction([zeros(14,1); -dtheta_driver], 'vars', {q, t, theta_driver, dtheta_driver}); 
Ctt_f = matlabFunction([zeros(14,1); -ddtheta_driver], 'vars', {q, t, theta_driver, dtheta_driver, ddtheta_driver}); 
Cqt_f = @(q,t,th,dth,ddth) zeros(15,15); 
Cqqdq_f = matlabFunction(jacobian(jacobian(C,q)*dq,q), 'vars', {q, dq, t, theta_driver}); 

%% Simulation loop prep
time = t_start:dt:t_stop;
qSol = NaN(15,length(time)); 
dqSol = qSol; 
ddqSol = qSol; 
Qc = NaN(15,length(time));
v_c_sol = NaN(2, length(time)); 
v_p_sol = NaN(2, length(time)); 

% give the solver some decent starting guesses so it doesn't get lost
q0 = zeros(15,1); 
q0(1:3) = [0; 0; th1_0]; 
q0(4:6) = [-L_vals(1) + 0.5*L_vals(5)*cos(th2_0); 0.5*L_vals(5)*sin(th2_0); th2_0]; 
q0(10:12) = [L_vals(3) + 0.5*L_vals(8)*cos(th4_0); 0.5*L_vals(8)*sin(th4_0); th4_0];

%% Run the simulation
for i = 1:length(time)
    % calc the sine wave for the motor
    th_val   = A_theta * sin(2 * pi * freq * time(i)); 
    dth_val  = A_theta * 2 * pi * freq * cos(2 * pi * freq * time(i));
    ddth_val = -A_theta * (2 * pi * freq)^2 * sin(2 * pi * freq * time(i));
    
    C_step     = @(qq, tt) C_f(qq, tt, th_val);
    Cq_step    = @(qq, tt) Cq_f(qq, tt, th_val);
    Ct_step    = @(qq, tt) Ct_f(qq, tt, th_val, dth_val);
    Cqt_step   = @(qq, tt) Cqt_f(qq, tt, th_val, dth_val, ddth_val);
    Ctt_step   = @(qq, tt) Ctt_f(qq, tt, th_val, dth_val, ddth_val);
    Cqqdq_step = @(qq, dqq) Cqqdq_f(qq, dqq, time(i), th_val); 
    
    if i == 1
        qSol(:,1) = nietraphson(q0, 0, C_step, Cq_step, 1e-4, 1e-4, 200); 
    else
        qSol(:,i) = nietraphson(qSol(:,i-1), time(i), C_step, Cq_step, 1e-4, 1e-4, 200); 
    end
    
    dqSol(:,i) = velocities(qSol(:,i), time(i), Cq_step, Ct_step);
    ddqSol(:,i) = accelerations(qSol(:,i), dqSol(:,i), time(i), Cq_step, Cqt_step, Ctt_step, Cqqdq_step);
    
    lambda = lagrangeMultipliers(qSol(:,i), ddqSol(:,i), time(i), Cq_step, M); 
    Qc(:,i) = -Cq_step(qSol(:,i), time(i))' * lambda; 
    
    v_p_sol(:,i) = Jp_f(qSol(:,i)) * dqSol(:,i);
    v_c_sol(:,i) = Jc_f(qSol(:,i)) * dqSol(:,i);
end

%% Stress and safety factors
% diff the tip speeds to get acceleration
a_c_sol = [diff(v_c_sol, 1, 2) / dt, [0;0]]; 
a_p_sol = [diff(v_p_sol, 1, 2) / dt, [0;0]];

% --- NEW: JPE Superposition Stress Framework ---
% Pass the positions (qSol) and forces (Qc) to extract both bending AND axial loads
[s_bend, s_axial, s_total, SF] = calculateLinkStresses(qSol, Qc, E_tpu, b_flex, h_flex, D_flex, TPU_yield);

%% Activation force via virtual work
I_flex_array = (b_flex .* h_flex.^3) / 12; 
k_spring = (E_tpu .* I_flex_array) / L_notch;
F_pad_required = zeros(1, length(time));
for i = 1:length(time)
    q = qSol(:, i);
    dq = dqSol(:, i);
    
    th1 = q(3); th2 = q(6); th3 = q(9); th4 = q(12); th5 = q(15);
    dth1 = dq(3); dth2 = dq(6); dth3 = dq(9); dth4 = dq(12); dth5 = dq(15);
    
    th1_0 = qSol(3,1); th2_0 = qSol(6,1); th3_0 = qSol(9,1); 
    th4_0 = qSol(12,1); th5_0 = qSol(15,1);
    
    bend_13 = abs((th3 - th1) - (th3_0 - th1_0)); 
    bend_23 = abs((th3 - th2) - (th3_0 - th2_0)); 
    bend_15 = abs((th5 - th1) - (th5_0 - th1_0)); 
    bend_45 = abs((th5 - th4) - (th5_0 - th4_0)); 
    
    tau_13 = k_spring(1) * bend_13; 
    tau_23 = k_spring(2) * bend_23; 
    tau_15 = k_spring(4) * bend_15; 
    tau_45 = k_spring(5) * bend_45; 
    
    w_13 = abs(dth3 - dth1);
    w_23 = abs(dth3 - dth2);
    w_15 = abs(dth5 - dth1);
    w_45 = abs(dth5 - dth4);
    
    Power_internal = (tau_13 * w_13) + (tau_23 * w_23) + (tau_15 * w_15) + (tau_45 * w_45);
    v_pad_x = abs(v_p_sol(1, i)); 
    v_pad_x = max(v_pad_x, 0.002);
    F_pad_required(i) = Power_internal / v_pad_x;
end

%% Outputs and plots
% Printing the separated axial and bending stresses as requested
printSystemParameters(L_vals, b_flex, h_flex, L_notch, scale, s_axial, s_bend, SF);

plotConfiguration(qSol, time, L_vals, scale, 20); 
plotVelocitiesAccelerations(dqSol, ddqSol, time, v_c_sol, v_p_sol, a_c_sol, a_p_sol);
plotReactionForces(Qc, time);

figure('Name', 'Required Activation Force', 'Color', 'w');
plot(time, F_pad_required, 'LineWidth', 3, 'Color', [0.85 0.325 0.098]);
grid on; ylim([0, 15]); 
title('Force required at pad to unfold wheel');
xlabel('Time (s)'); ylabel('Push force (N)');
yline(12.75, 'r--', 'Robot weight (1.3kg)');