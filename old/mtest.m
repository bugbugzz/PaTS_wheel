clear all, close all, clc
%% 1. INITIALISATION & PARAMETERS'
% L = [TopBase; TopExt; ScoopWidth; BotArm; ScoopHeight] 
L = [1.0; 1; 1.0; 0.5]; 
rho = [7870; 7870; 7870; 7870];
D = [100; 100; 100; 100];
m = rho .* L .* pi .* ( (D./1000)./2 ).^2;
I = (1/12) .* m .* L.^2;

% --- FIXED RIGHT-ANGLE INITIALIZATION ---
% 1. Calculate the exact Hypotenuse angle
thetaA_init = atan2(-L(4), L(3)); % Angle to point from (0,0) to (1.0, -0.8)
thetaB_init = deg2rad(0);                        % Bottom arm is horizontal
th3_init = -pi/2+thetaB_init;                       % Scoop is vertical

% 2. Calculate the Hypotenuse length for CM2 placement
hypot_len = sqrt(L(3)^2 + L(4)^2); 
L(1)=hypot_len/2;
L(2)=hypot_len/2;

q0 = zeros(12,1);

% Body 1: Upper Base (CM at half of L1)
q0(1:3) = [0.5*L(1)*cos(thetaA_init); 0.5*L(1)*sin(thetaA_init); thetaA_init];

% Body 2: Upper Extension (CM is at the total hypotenuse minus half of L2)
% This ensures its "tip" lands exactly at (1.0, -0.8)
q0(4:6) = [(hypot_len - 0.5*L(2))*cos(thetaA_init); (hypot_len - 0.5*L(2))*sin(thetaA_init); thetaA_init];

% Body 3: Scoop (CM midway between elbow and tip)
q0(7:9) = [L(3)*cos(thetaB_init); -0.5*L(4)*sin(thetaB_init); th3_init];

% Body 4: Bottom Arm (CM at half of L3)
q0(10:12) = [0.5*L(3)*cos(thetaB_init); 0.5*L(3)*sin(thetaB_init); thetaB_init];

% 3. Update the initial driver length
initial_len_val = hypot_len - 0.5*L(1) - 0.5*L(2);


%% 2. SIMULATION SETTINGS
freq = 0.3;                     % 0.5 Hz = 2 seconds per cycle
A_w  = deg2rad(15);             % Lift amplitude
A_V  = 0.3;                    % Scoop extension amplitude

t_start = 0; dt = 0.01; t_stop = 8.0; % Longer time to see the slow motion
tol_I = 1e-7; tol_step = 1e-7; maxI = 150;
plot_conf_over_time = 1; 
plot_stepsize = 10;

%% 3. SYMBOLIC DEFINITION
q = sym('q',[12 1]); dq = sym('dq',[12 1]); syms t

x1=q(1); y1=q(2); th1=q(3); x2=q(4); y2=q(5); th2=q(6);
x3=q(7); y3=q(8); th3=q(9); x4=q(10);y4=q(11);th4=q(12);

% Constraints
eq1 = x1 - 0.5*L(1)*cos(th1); eq2 = y1 - 0.5*L(1)*sin(th1);
eq3 = x4 - 0.5*L(3)*cos(th4); eq4 = y4 - 0.5*L(3)*sin(th4);
eq5 = th1 - th2; 
eq6 = (y2 - y1)*cos(th1) - (x2 - x1)*sin(th1);
eq7 = (x2 + 0.5*L(2)*cos(th2)) - (x3 - (L(4)/2)*sin(th3));
eq8 = (y2 + 0.5*L(2)*sin(th2)) - (y3 + (L(4)/2)*cos(th3));
eq9 = (x4 + 0.5*L(3)*cos(th4)) - (x3 + (L(4)/2)*sin(th3));
eq10= (y4 + 0.5*L(3)*sin(th4)) - (y3 - (L(4)/2)*cos(th3));

% --- PERIODIC SINE DRIVERS ---
% eq11: Bottom arm lift and return
eq11 = th4 - (thetaB_init + A_w * sin(2 * pi * freq * t)); 

% eq12: Piston extension and retraction
current_len = (x2 - x1)*cos(th1) + (y2 - y1)*sin(th1);
eq12 = current_len - (initial_len_val + A_V * sin(2 * pi * freq * t));
C = [eq1; eq2; eq3; eq4; eq5; eq6; eq7; eq8; eq9; eq10; eq11; eq12];
Cq = jacobian(C,q); Ct = diff(C,t); Cqt = diff(Cq,t); Ctt = diff(Ct,t);
Cqqdq = jacobian(Cq*dq,q);

% Function conversion
C_f = matlabFunction(C,'vars',[{q},{t}]);
Cq_f = matlabFunction(Cq,'vars',[{q},{t}]);
Ct_f = matlabFunction(Ct,'vars',[{q},{t}]);
Cqt_f = matlabFunction(Cqt,'vars',[{q},{t}]);
Ctt_f = matlabFunction(Ctt,'vars',[{q},{t}]);
Cqqdq_f = matlabFunction(Cqqdq,'vars',[{q},{dq},{t}]);

%% 4. SIMULATION
time = t_start:dt:t_stop;
qSol = NaN(12,length(time)); dqSol = qSol; ddqSol = qSol;
lambdaSol = qSol; Qc = NaN(12,length(time));
M = zeros(12);
for i = 1:4
    ind = 3*(i-1)+1:3*i;
    M(ind,ind) = diag([m(i), m(i), I(i)]);
end

% Initial step calculation
% Save data for the first time step
% 
% Calculate the initial state at t=0
qSol(:,1) = nietraphson(q0, 0, C_f, Cq_f, tol_I, tol_step, maxI);
dqSol(:,1) = velocities(qSol(:,1), 0, Cq_f, Ct_f);

% Calculate initial acceleration and lambda
ddqSol(:,1) = accelerations(qSol(:,1), dqSol(:,1), time(1), Cq_f, Cqt_f, Ctt_f, Cqqdq_f);
lambdaSol(:,1) = lagrangeMultipliers(qSol(:,1), ddqSol(:,1), time(1), Cq_f, M);
Qc(:,1) = reactionForces(qSol(:,1),time(1),lambdaSol(:,1),L,Cq_f);  % Save the reaction forces at time(1)



for i = 2:length(time)
    qSol(:,i) = nietraphson(qSol(:,i-1), time(i), C_f, Cq_f, tol_I, tol_step, maxI);
    dqSol(:,i) = velocities(qSol(:,i),time(i),Cq_f,Ct_f);
    ddqSol(:,i) = accelerations(qSol(:,i),dqSol(:,i),time(i),Cq_f,Cqt_f,Ctt_f,Cqqdq_f);
    
    lambdaSol(:,i) = lagrangeMultipliers(qSol(:,i),ddqSol(:,i),time(i),Cq_f,M);
    Qc(:,i) = reactionForces(qSol(:,i),time(i),lambdaSol(:,i),L,Cq_f);
end

%% plotting
% Plot the configuration over time
if plot_conf_over_time ~= 0     % Only execute plot if wanted
    plotConfiguration(qSol,time,L,plot_stepsize)
end

% Plot the velocities and acclerations
plotVelocitiesAccelerations(dqSol,ddqSol,time)

% Plot the forces and moments
plotReactionForces(Qc,time)   

T_lift = -lambdaSol(11, :); 

% 2. Extension Force
% The driver is Constraint 12.
% Similarly, the force is -lambda(12).
F_extension = -lambdaSol(12, :);

% 3. Save to file
save('GraafmachineForces.mat', 'T_lift', 'F_extension');

disp('Corrected Forces Saved. T_lift should now be non-zero.');
% Optional: Check the value
fprintf('Average Lift Torque: %f Nm\n', mean(T_lift));
