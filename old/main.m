clear all, close all, clc
%% 1. INITIALISATION & PARAMETERS'
% L = [TopBase; TopExt; ScoopWidth; BotArm; ScoopHeight] 
L = [1.0; 1.5; 1.0; 0.8]; 
rho = [7870; 7870; 7870; 7870];
D = [100; 100; 100; 100];
m = rho .* L .* pi .* ( (D./1000)./2 ).^2;
I = (1/12) .* m .* L.^2;

% --- GEOMETRIC INITIALIZATION (Angled Config) ---
thetaB_init = deg2rad(45); 
th3_init = -deg2rad(27); 

TipF_x = L(3) * cos(thetaB_init);
TipF_y = L(3) * sin(thetaB_init);
x3_init = TipF_x;              
y3_init = TipF_y + L(4)/2;     
TipE_x = x3_init - (L(4)/2)*sin(th3_init);
TipE_y = y3_init + (L(4)/2)*cos(th3_init);
thetaA_init = atan2(TipE_y, TipE_x); 

q0 = zeros(12,1);
q0(1:3) = [0.5*L(1)*cos(thetaA_init); 0.5*L(1)*sin(thetaA_init); thetaA_init];
q0(4:6) = [TipE_x - 0.5*L(2)*cos(thetaA_init); TipE_y - 0.5*L(2)*sin(thetaA_init); thetaA_init];
q0(7:9) = [x3_init; y3_init; th3_init];
q0(10:12) = [0.5*L(3)*cos(thetaB_init); 0.5*L(3)*sin(thetaB_init); thetaB_init];
dx = q0(4) - q0(1); dy = q0(5) - q0(2);
initial_len_val = sqrt(dx^2 + dy^2); 

%% 2. SIMULATION SETTINGS
% --- SLIDER CRANK STYLE: Constant Velocities ---
w_target = -0.2;  
V_target = -0.2;  
t_start = 0; dt = 0.01; t_stop = 2.5; 
tol_I = 1e-9; tol_step = 1e-9; maxI = 100;
plot_conf_over_time = 1; 
plot_stepsize = 20; 

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

% --- SLIDER CRANK STYLE DRIVERS: omega*t ---
% This matches eq6 = q(3) - omega*t from your example
eq11 = th4 - (thetaB_init + w_target * t); 
current_len = (x2 - x1)*cos(th1) + (y2 - y1)*sin(th1);
eq12 = current_len - (initial_len_val + V_target * t); 

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
