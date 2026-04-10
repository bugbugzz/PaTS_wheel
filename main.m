clear all; close all; clc;

%% 1. ALIGNED PARAMETERS
scale = 2.57; % Scales the 100mm base to 257mm
L_vals = ([50; 25; 50; 25; 25; 25; 25; 25] * scale) / 1000; 

% Flexure Design (The TPU Notches)
b_flex = 20 / 1000; % 20mm wide (wheel depth)
h_flex = 3 / 1000;  % 3mm thick (notch thickness)
TPU_yield = 20e6;   % 20 MPa yield strength for TPU 95A

% Mass & Inertia Properties
rho_tpu = 1200; D = 10 / 1000;
m = rho_tpu * L_vals(1:5) * pi * (D/2)^2;
I = (1/12) * m .* L_vals(1:5).^2;

M = zeros(15);
for i = 1:5
    ind = 3*(i-1)+1:3*i;
    M(ind,ind) = diag([m(i), m(i), I(i)]);
end

%% 2. KINEMATICS & SYMBOLIC SETUP
q = sym('q',[15 1]); dq = sym('dq',[15 1]); 
syms t theta_driver dtheta_driver ddtheta_driver 
x1=q(1); y1=q(2); th1=q(3); x2=q(4); y2=q(5); th2=q(6);
x3=q(7); y3=q(8); th3=q(9); x4=q(10);y4=q(11);th4=q(12); x5=q(13);y5=q(14);th5=q(15);

% Tip Kinematics (c = Claw, p = Pad)
b3_sym = [x1 + L_vals(4)*cos(th1 + pi - 1.4); y1 + L_vals(4)*sin(th1 + pi - 1.4)];
a2_sym = [x1 + L_vals(2)*cos(th1); y1 + L_vals(2)*sin(th1)];
tip_height = L_vals(6); 
c_sym = [b3_sym(1) - tip_height*sin(th3); b3_sym(2) + tip_height*cos(th3)]; 
p_sym = [a2_sym(1) - tip_height*sin(th5); a2_sym(2) + tip_height*cos(th5)];

Jp_f = matlabFunction(jacobian(p_sym, q), 'vars', {q});
Jc_f = matlabFunction(jacobian(c_sym, q), 'vars', {q});

% Constraints
th1_0 = 0.7; 
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

C_f = matlabFunction(C, 'vars', {q, t, theta_driver});
Cq_f = matlabFunction(jacobian(C,q), 'vars', {q, t, theta_driver});
Ct_f = matlabFunction([zeros(14,1); -dtheta_driver], 'vars', {q, t, theta_driver, dtheta_driver});
Ctt_f = matlabFunction([zeros(14,1); -ddtheta_driver], 'vars', {q, t, theta_driver, dtheta_driver, ddtheta_driver});
Cqt_f = @(q,t,th,dth,ddth) zeros(15,15);
Cqqdq_f = matlabFunction(jacobian(jacobian(C,q)*dq,q), 'vars', {q, dq, t, theta_driver});

%% 3. PREPARE SIMULATION ARRAYS
t_start = 0; dt = 0.005; t_stop = 4; time = t_start:dt:t_stop;
qSol = NaN(15,length(time)); dqSol = qSol; ddqSol = qSol; Qc = NaN(15,length(time));
v_c_sol = NaN(2, length(time)); v_p_sol = NaN(2, length(time)); 

% Initial Guesses
th2_0 = 2.0; th4_0 = 1.0; 
q0 = zeros(15,1); q0(1:3) = [0; 0; th1_0]; 
q0(4:6) = [-L_vals(1) + 0.5*L_vals(5)*cos(th2_0); 0.5*L_vals(5)*sin(th2_0); th2_0]; 
q0(10:12) = [L_vals(3) + 0.5*L_vals(8)*cos(th4_0); 0.5*L_vals(8)*sin(th4_0); th4_0];

%% 4. RUN SIMULATION LOOP
freq = 0.5; A_theta = deg2rad(20);

for i = 1:length(time)
    th_val   = A_theta * sin(2 * pi * freq * time(i)); 
    dth_val  = A_theta * 2 * pi * freq * cos(2 * pi * freq * time(i));
    ddth_val = -A_theta * (2 * pi * freq)^2 * sin(2 * pi * freq * time(i));
    
    C_step     = @(qq, tt) C_f(qq, tt, th_val);
    Cq_step    = @(qq, tt) Cq_f(qq, tt, th_val);
    Ct_step    = @(qq, tt) Ct_f(qq, tt, th_val, dth_val);
    Cqt_step   = @(qq, tt) Cqt_f(qq, tt, th_val, dth_val, ddth_val);
    Ctt_step   = @(qq, tt) Ctt_f(qq, tt, th_val, dth_val, ddth_val);
    Cqqdq_step = @(qq, dqq) Cqqdq_f(qq, dqq, time(i), th_val); 

    % Solve Kinematics
    if i == 1, qSol(:,1) = nietraphson(q0, 0, C_step, Cq_step, 1e-7, 1e-7, 150);
    else, qSol(:,i) = nietraphson(qSol(:,i-1), time(i), C_step, Cq_step, 1e-7, 1e-7, 150); end
    
    dqSol(:,i) = velocities(qSol(:,i), time(i), Cq_step, Ct_step);
    ddqSol(:,i) = accelerations(qSol(:,i), dqSol(:,i), time(i), Cq_step, Cqt_step, Ctt_step, Cqqdq_step);

    % Solve Dynamics & Internal Forces
    lambda = lagrangeMultipliers(qSol(:,i), ddqSol(:,i), time(i), Cq_step, M);
    Qc(:,i) = -Cq_step(qSol(:,i), time(i))' * lambda;
    
    % Track Tips
    v_p_sol(:,i) = Jp_f(qSol(:,i)) * dqSol(:,i);
    v_c_sol(:,i) = Jc_f(qSol(:,i)) * dqSol(:,i);
end

%% 5. DATA PROCESSING & STRESS ANALYSIS
% Numerical Tip Accelerations
a_c_sol = [diff(v_c_sol, 1, 2) / dt, [0;0]]; 
a_p_sol = [diff(v_p_sol, 1, 2) / dt, [0;0]];

% Calculate Internal Stress (Axial vs Bending)
[~, s_axial, s_bend, SF] = calculateLinkStresses(Qc, b_flex, h_flex, TPU_yield, L_vals);

% Print the Report to Console
fprintf('\n--- STRESS REPORT: 1.3kg Robot Climbing 18cm Step ---\n');
fprintf('%-18s | %-12s | %-12s | %-8s\n', 'Flexure Node', 'Axial (MPa)', 'Bending (MPa)', 'SF');
fprintf('------------------------------------------------------------\n');

% Renamed to reflect the exact joints/notches being calculated
node_names = {'Coupler Hinge', 'Claw Base Hinge', 'Claw Tip Hinge', 'Pad Base Hinge', 'Pad Tip Hinge'};

for i = 1:5
    fprintf('%-18s | %-12.4f | %-12.4f | %-8.2f\n', ...
            node_names{i}, max(s_axial(i,:))/1e6, max(s_bend(i,:))/1e6, SF(i));
end

%% 6. VISUALIZATION & PLOTTING
plotConfiguration(qSol, time, L_vals, 20);
plotVelocitiesAccelerations(dqSol, ddqSol, time, v_c_sol, v_p_sol, a_c_sol, a_p_sol);
plotReactionForces(Qc, time); 

printSystemParameters(L_vals, b_flex, h_flex, scale);
disp('Simulation Complete.');