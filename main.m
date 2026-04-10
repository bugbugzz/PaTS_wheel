clear all, close all, clc

%% 1. INITIALISATION & PARAMETERS (PaTS-Wheel 5-Body System)
% Link lengths: L_vals = [Lb1; Lb4; La1; La2; Lb2; Lb3; La3; La4]
% Estimated based on c1 ≈ 59mm scale.
% --- NEW REALISTIC 18cm CLIMBER PARAMETERS ---
scale = 2.6; % Scales the 100mm design to a 260mm design
L_vals = ([50; 25; 50; 25; 25; 25; 25; 25] * scale) / 1000; 

% FLEXURE DESIGN (The "Hinges")
b_flex = 15 / 1000; % 15mm wide (depth of the wheel)
h_flex = 2.5 / 1000; % 2.5mm thick (the part that bends)
rho_tpu = 1200; % [kg/m^3] TPU density
D = 10 / 1000;  % [m] 10mm thickness/diameter
 
% Calculate Mass and Inertia for 5 bodies
m = rho_tpu * L_vals(1:5) * pi * (D/2)^2;
I = (1/12) * m .* L_vals(1:5).^2;

% Initialize Mass Matrix M (15x15)
M = zeros(15);
for i = 1:5
    ind = 3*(i-1)+1:3*i;
    M(ind,ind) = diag([m(i), m(i), I(i)]);
end

% Initial angles (radians) - Coupler centered at 0.7 rad (~40 deg)
th1_0 = 0.7; th2_0 = 2.0; th4_0 = 1.0; 

% Initialize q0 (15 coordinates for 5 bodies)
q0 = zeros(15,1);
q0(1:3)   = [0; 0; th1_0]; % Body 1: Coupler (CM at pivot 0,0)
q0(4:6)   = [-L_vals(1) + 0.5*L_vals(5)*cos(th2_0); 0.5*L_vals(5)*sin(th2_0); th2_0]; % Body 2: Claw Support
q0(10:12) = [L_vals(3) + 0.5*L_vals(8)*cos(th4_0); 0.5*L_vals(8)*sin(th4_0); th4_0]; % Body 4: Pad Support
% Note: Bodies 3 and 5 start at 0,0,0; Newton-Raphson will snap them into place on step 1.

%% 2. SIMULATION SETTINGS
freq = 0.5;                           % 0.5 Hz oscillation
A_theta = deg2rad(20);                % Coupler oscillates +/- 10 degrees
t_start = 0; dt = 0.005; t_stop = 4;  % Time settings
tol_I = 1e-7; tol_step = 1e-7; maxI = 150;
plot_conf_over_time = 2; 
plot_stepsize = 20;                   % Adjusted for new dt

%% 3. SYMBOLIC DEFINITION
q = sym('q',[15 1]); dq = sym('dq',[15 1]); syms t 

% Driver variable for the Coupler angle
syms theta_driver dtheta_driver ddtheta_driver 

% Extract variables for easy reading
x1=q(1); y1=q(2); th1=q(3); x2=q(4); y2=q(5); th2=q(6);
x3=q(7); y3=q(8); th3=q(9); x4=q(10);y4=q(11);th4=q(12);
x5=q(13);y5=q(14);th5=q(15);

% --- DEFINE FUNCTIONAL POINTS (c and p) ---
% Define the inner joints (matching the plot geometry)
b3_sym = [x1 + L_vals(4)*cos(th1 + pi - 1.4); y1 + L_vals(4)*sin(th1 + pi - 1.4)];
a2_sym = [x1 + L_vals(2)*cos(th1); y1 + L_vals(2)*sin(th1)];

tip_height = L_vals(6); 

% Define the Tips (Claw = c, Pad = p) pointing UP and INWARD
c_sym = [b3_sym(1) - tip_height*sin(th3); b3_sym(2) + tip_height*cos(th3)]; 
p_sym = [a2_sym(1) - tip_height*sin(th5); a2_sym(2) + tip_height*cos(th5)];

% Calculate the Jacobians (velocity multipliers) for points c and p
Jp_sym = jacobian(p_sym, q); % How the Pad moves relative to the whole system
Jc_sym = jacobian(c_sym, q); % How the Claw moves relative to the whole system

% Convert to MATLAB functions for the simulation loop
Jp_f = matlabFunction(Jp_sym, 'vars', {q});
Jc_f = matlabFunction(Jc_sym, 'vars', {q});

% --- CONSTRAINTS ---
% 1. Body 1 pivot at Ground (0,0)
eq1 = x1; eq2 = y1;
% 2. Body 2 pivot at Ground b1 (-Lb1, 0)
eq3 = x2 - 0.5*L_vals(5)*cos(th2) - (-L_vals(1));
eq4 = y2 - 0.5*L_vals(5)*sin(th2) - 0;
% 3. Body 4 pivot at Ground a4 (La1, 0)
eq5 = x4 - 0.5*L_vals(8)*cos(th4) - L_vals(3);
eq6 = y4 - 0.5*L_vals(8)*sin(th4) - 0;
% 4. Body 1-3 joint at b3 (Fixed: True Rigid V-Shape!)
eq7 = (x1 + L_vals(4)*cos(th1 + pi - 1.4)) - (x3 + 0.5*L_vals(6)*cos(th3));
eq8 = (y1 + L_vals(4)*sin(th1 + pi - 1.4)) - (y3 + 0.5*L_vals(6)*sin(th3));
% 5. Body 2-3 joint at b2
eq9 = (x2 + 0.5*L_vals(5)*cos(th2)) - (x3 - 0.5*L_vals(6)*cos(th3));
eq10= (y2 + 0.5*L_vals(5)*sin(th2)) - (y3 - 0.5*L_vals(6)*sin(th3));
% 6. Body 1-5 joint at a2
eq11 = (x1 + L_vals(2)*cos(th1)) - (x5 - 0.5*L_vals(6)*cos(th5));
eq12 = (y1 + L_vals(2)*sin(th1)) - (y5 - 0.5*L_vals(6)*sin(th5));
% 7. Body 4-5 joint at a3
eq13 = (x4 + 0.5*L_vals(8)*cos(th4)) - (x5 + 0.5*L_vals(6)*cos(th5));
eq14 = (y4 + 0.5*L_vals(8)*sin(th4)) - (y5 + 0.5*L_vals(6)*sin(th5));
% 8. Driver Constraint (Control the Coupler angle)
eq15 = th1 - (th1_0 + theta_driver); 

C = [eq1; eq2; eq3; eq4; eq5; eq6; eq7; eq8; eq9; eq10; eq11; eq12; eq13; eq14; eq15];

% --- MATRIX FUNCTIONS ---
% Create MATLAB functions for the solver
C_f = matlabFunction(C, 'vars', {q, t, theta_driver});
Cq_f = matlabFunction(jacobian(C,q), 'vars', {q, t, theta_driver});

% Velocity constraints (Ct) -> only eq15 depends explicitly on time/driver
Ct_sym = [zeros(14,1); -dtheta_driver]; 
Ct_f = matlabFunction(Ct_sym, 'vars', {q, t, theta_driver, dtheta_driver});

% Acceleration constraints (Ctt)
Ctt_sym = [zeros(14,1); -ddtheta_driver];
Ctt_f = matlabFunction(Ctt_sym, 'vars', {q, t, theta_driver, dtheta_driver, ddtheta_driver});

% Cqt is a 15x15 zero matrix because Jacobian coefficients don't explicitly change with time
Cqt_f = @(q,t,th,dth,ddth) zeros(15,15);

% Cqqdq for Coriolis/Centripetal accelerations
Cqqdq_f = matlabFunction(jacobian(jacobian(C,q)*dq,q), 'vars', {q, dq, t, theta_driver});


%% 4. PREPARE SIMULATION ARRAYS
time = t_start:dt:t_stop;
qSol = NaN(15,length(time)); dqSol = qSol; ddqSol = qSol;
lambdaSol = qSol; Qc = NaN(15,length(time));

F_claw_output = NaN(1, length(time)); 
v_c_sol = NaN(2, length(time)); % NEW: Array for Claw Tip X/Y Velocity
v_p_sol = NaN(2, length(time)); % NEW: Array for Pad Tip X/Y Velocity


%% 5. RUN SIMULATION
for i = 1:length(time)
    
    % Simple harmonic motion for the driving coupler
    % Simulates the pad being pressed and released
    th_val   = A_theta * sin(2 * pi * freq * time(i)); 
    dth_val  = A_theta * 2 * pi * freq * cos(2 * pi * freq * time(i));
    ddth_val = -A_theta * (2 * pi * freq)^2 * sin(2 * pi * freq * time(i));
    
    % Update function handles with current driver values
    C_step     = @(qq, tt) C_f(qq, tt, th_val);
    Cq_step    = @(qq, tt) Cq_f(qq, tt, th_val);
    Ct_step    = @(qq, tt) Ct_f(qq, tt, th_val, dth_val);
    
    Cqt_step   = @(qq, tt) Cqt_f(qq, tt, th_val, dth_val, ddth_val);
    Ctt_step   = @(qq, tt) Ctt_f(qq, tt, th_val, dth_val, ddth_val);
    
    Cqqdq_step = @(qq, dqq) Cqqdq_f(qq, dqq, time(i), th_val); 

    % 1. Solve Position (Newton-Raphson)
    if i == 1
        qSol(:,1) = nietraphson(q0, 0, C_step, Cq_step, tol_I, tol_step, maxI);
    else
        qSol(:,i) = nietraphson(qSol(:,i-1), time(i), C_step, Cq_step, tol_I, tol_step, maxI);
    end

    % 2. Solve Velocities
    dqSol(:,i) = velocities(qSol(:,i), time(i), Cq_step, Ct_step);

    % 3. Solve Accelerations
    ddqSol(:,i) = accelerations(qSol(:,i), dqSol(:,i), time(i), ...
                  Cq_step, Cqt_step, Ctt_step, Cqqdq_step);


% 4. Solve Dynamics (Forces)
    lambdaSol(:,i) = lagrangeMultipliers(qSol(:,i), ddqSol(:,i), time(i), Cq_step, M);
    
    % CALCULATE REACTION FORCES DIRECTLY
    % Qc represents the internal force vector [Fx, Fy, Moment] for all 5 bodies
    Qc(:,i) = -Cq_step(qSol(:,i), time(i))' * lambdaSol(:,i);    % --- 5. CALCULATE GRIPPING FORCE VIA VIRTUAL WORK ---
% --- 5. CALCULATE TIP KINEMATICS & GRIPPING FORCE ---
    Jp_step = Jp_f(qSol(:,i));
    Jc_step = Jc_f(qSol(:,i));
    
    % Track the exact X and Y velocity of the tips [vx; vy]
    v_p_sol(:,i) = Jp_step * dqSol(:,i);
    v_c_sol(:,i) = Jc_step * dqSol(:,i);
    
    % Calculate gripping force (assuming 10N horizontal push on pad)
    Force_Pad_X = 10; 
    F_claw_output(i) = abs((Force_Pad_X * v_p_sol(1,i)) / v_c_sol(1,i));
end



%% 6. POST-PROCESSING & PLOTTING

if plot_conf_over_time ~= 0     
    plotConfiguration(qSol, time, L_vals, plot_stepsize);
end

% ---> MAKE SURE THIS LINE HAS 5 VARIABLES IN THE BRACKETS <---
plotVelocitiesAccelerations(dqSol, ddqSol, time, v_c_sol, v_p_sol);

plotReactionForces(Qc, time);   
postProcessResults(time, F_claw_output, lambdaSol, qSol, dqSol, Qc);
% Save results

% 
% %% 7. DETAILED STRESS & STRENGTH ANALYSIS
% TPU_yield = 20e6;
% 
% [s_total, s_axial, s_bend, SF] = calculateLinkStresses(Qc, b_flex, h_flex, TPU_yield);
% 
% fprintf('\n--- DETAILED STRESS REPORT (1.3kg Robot) ---\n');
% fprintf('%-18s | %-12s | %-12s | %-8s\n', 'Body Name', 'Axial (MPa)', 'Bending (MPa)', 'SF');
% fprintf('------------------------------------------------------------\n');
% 
% body_names = {'Coupler', 'Claw Support', 'Claw Tip', 'Pad Support', 'Pad Tip'};
% for i = 1:5
%     max_a = max(s_axial(i,:)) / 1e6;
%     max_b = max(s_bend(i,:)) / 1e6;
%     fprintf('%-18s | %-12.4f | %-12.4f | %-8.2f\n', ...
%             body_names{i}, max_a, max_b, SF(i));
% end