clear all, close all, clc

%% 1. INITIALISATION & PARAMETERS
% L = [TopBase; TopExt; ScoopWidth; BotArm; ScoopHeight]
L = [1.0; 1.5; 1.0; 0.8]; 
rho = [7870; 7870; 7870; 7870];                % The density of the elements [kg/m^3]
D = [100; 100; 100; 100];
m = rho .* L .* pi .* ( (D./1000)./2 ).^2;                      % Mass of the elements [kg]
I = (1/12) .* m .* L.^2;                    % Second moment of mass about the bodies centre of mass [kg*m^2]

% 1. Master Angle: Bottom Arm (Theta B)
thetaB_init = deg2rad(45); 

% 2. Calculate Bottom Arm Tip (Point F)
TipF_x = L(3) * cos(thetaB_init);
TipF_y = L(3) * sin(thetaB_init);

% 3. Calculate Scoop Position (Start Vertical)
th3_init = -deg2rad(27); 
x3_init = TipF_x;              % Scoop Center X
y3_init = TipF_y + L(4)/2;     % Scoop Center Y

% 4. Calculate Top of Scoop (Point E)
% This is the target that the Top Arm must reach
TipE_x = x3_init - (L(4)/2)*sin(th3_init);
TipE_y = y3_init + (L(4)/2)*cos(th3_init);

% 5. Calculate Top Arm Angle (Theta A)
% It must point exactly from Origin (0,0) to Point E
thetaA_init = atan2(TipE_y, TipE_x); 

% 6. Build the q0 vector
q0 = zeros(12,1);

% Body 1 (Top Base)
q0(3) = thetaA_init;
q0(1) = 0.5 * L(1) * cos(thetaA_init);
q0(2) = 0.5 * L(1) * sin(thetaA_init);

% Body 2 (Top Ext) - Aligned with Body 1
q0(6) = thetaA_init;
q0(4) = TipE_x - 0.5 * L(2) * cos(thetaA_init);
q0(5) = TipE_y - 0.5 * L(2) * sin(thetaA_init);

% Body 3 (Scoop)
q0(9) = th3_init;
q0(7) = x3_init;
q0(8) = y3_init;

% Body 4 (Bot Arm)
q0(12) = thetaB_init;
q0(10) = 0.5 * L(3) * cos(thetaB_init);
q0(11) = 0.5 * L(3) * sin(thetaB_init);

% --- CALCULATE INITIAL EXTENSION LENGTH ---
% Distance between Center 1 and Center 2 at start
dx = q0(4) - q0(1); 
dy = q0(5) - q0(2);
initial_len_val = sqrt(dx^2 + dy^2); 

%% 2. SIMULATION SETTINGS

w = -0.2;  % lift
V = -0.2;  % extension



t_start = 0;
dt = 0.001;
t_stop = 2.5; 
tol_I = 1e-9;
tol_step = 1e-9;
maxI = 100;

plot_conf_over_time = 1;    % If 0, the configuration over time will not be plotted
plot_stepsize = 20; 

%% determine the mass matrix per body  
M = zeros(length(q0));      % A matrix of zeros
for i = 1:length(q0)/3      % Add diagonal terms per body
    ind = 3*(i-1)+1:3*i;    % Index of current diagonal terms
    M(ind,ind) = diag([m(i), m(i), I(i)]);  % Add the diagonal terms to the mass matrix
end

%% 3. SYMBOLIC DEFINITION
q = sym('q',[12 1]); 
dq = sym('dq',[length(q0) 1]);      % Create the vector dq with length(q0) symbolic parameters  
syms t

% --- UNPACK VARIABLES ---
x1=q(1); y1=q(2); th1=q(3);   % Top Base
x2=q(4); y2=q(5); th2=q(6);   % Top Ext
x3=q(7); y3=q(8); th3=q(9);   % Scoop
x4=q(10);y4=q(11);th4=q(12);  % Bot Arm

% --- CONSTRAINT EQUATIONS ---

% 1. Pin A (Body 1 to Ground)
eq1 = x1 - 0.5*L(1)*cos(th1); %
eq2 = y1 - 0.5*L(1)*sin(th1);

% 2. Pin B (Body 4 to Ground)
eq3 = x4 - 0.5*L(3)*cos(th4);
eq4 = y4 - 0.5*L(3)*sin(th4);

% 3. Slider Joint (Body 1 & Body 2)
% Constraint A: Angles must be equal (Telescopic)
eq5 = th1 - th2; 
% Constraint B: Collinear (Centers lie on the same axis)
eq6 = (y2 - y1)*cos(th1) - (x2 - x1)*sin(th1);

% 4. Pin E (Body 2 to Scoop Top)
eq7 = (x2 + 0.5*L(2)*cos(th2)) - (x3 - (L(4)/2)*sin(th3));
eq8 = (y2 + 0.5*L(2)*sin(th2)) - (y3 + (L(4)/2)*cos(th3));

% 5. Pin F (Body 4 to Scoop Bottom)
eq9 = (x4 + 0.5*L(3)*cos(th4)) - (x3 + (L(4)/2)*sin(th3));
eq10= (y4 + 0.5*L(3)*sin(th4)) - (y3 - (L(4)/2)*cos(th3));

% --- DRIVERS ---

% Driver 1: Lift (Bottom Arm Rotation)
% Starts smoothly from thetaB_init
eq11 = th4 - (thetaB_init + w*t); 

% Driver 2: Extension (Slider Length)
% Starts smoothly from initial_len_val
current_len = (x2 - x1)*cos(th1) + (y2 - y1)*sin(th1);
eq12 = current_len - (initial_len_val + V*t); 

% Assemble Constraints
C = [eq1; eq2; eq3; eq4; eq5; eq6; eq7; eq8; eq9; eq10; eq11; eq12];

% Derivatives
% Take the derivatives of the constraint equations
Cq = jacobian(C,q);         % The Jacobian
Ct = diff(C,t);             % The derivative of the constraint with respect to time
Cqt = diff(Cq,t);           % The derivative of the Jacobian with respect to time
Ctt = diff(Ct,t);           % The second derivative of the constraints with respect to time
Cqqdq = jacobian(Cq*dq,q);  % The derivative of the Jacobian multiplied with the velocities

% Transform the symbolic formula's into functions
C_f = matlabFunction(C,'vars',[{q},{t}]);
Cq_f = matlabFunction(Cq,'vars',[{q},{t}]);
Ct_f = matlabFunction(Ct,'vars',[{q},{t}]);
Cqt_f = matlabFunction(Cqt,'vars',[{q},{t}]);
Ctt_f = matlabFunction(Ctt,'vars',[{q},{t}]);
Cqqdq_f = matlabFunction(Cqqdq,'vars',[{q},{dq},{t}]);

% %% 4. SIMULATION LOOP
% time = t_start:dt:t_stop;
% qSol = NaN(length(q0),length(time));
% qSol(:,1) = q0;
% 
% disp('Starting Simulation...');
% for i = 2:length(time)
%     % Call clean solver (assumed to be in rapshson.m)
%     qSol(:,i) = nietraphson(qSol(:,i-1), time(i), C_f, Cq_f, tol_I, tol_step, maxI);
% end
% disp('Simulation Complete.');
% 
% % calculate the velocities
% dqSol = NaN(length(q0),length(time));   % Empty matrix to save velocities over time
% for i = 1:length(time)
%    dqSol(:,i) = velocities(qSol(:,i),time(i),Cq_f,Ct_f);
% end
% 
% % calculate the accelerations
% ddqSol = NaN(length(q0),length(time));  % Empty matrix to save accelerations over time
% for i = 1:length(time)
%    ddqSol(:,i) = accelerations(qSol(:,i),dqSol(:,i),time(i),Cq_f,Cqt_f,Ctt_f,Cqqdq_f);
% end
% % %% 5. PLOTTING
% % if exist('nietplot','file')
% %     nietplot(qSol, time, L, plot_stepsize);
% % else
% %     disp('Plot function not found.');
% % end

%% simulation
% Initialize
time = t_start:dt:t_stop;                 % Vector of all time instances
qSol = NaN(length(q0),length(time));      % Empty matrix to save positions over time
dqSol = NaN(length(q0),length(time));     % Empty matrix to save velocities over time
ddqSol = NaN(length(q0),length(time));    % Empty matrix to save accelerations over time
lambdaSol = NaN(length(q0),length(time)); % Empty matrix to save lagrange multipliers over time
Qc = NaN(12,length(time)-1);              % Empty matrix to save reaction forces over time

% Save data for the first time step
qSol(:,1) = q0;                                         % Save configuration at time(1)
dqSol(:,1) = velocities(qSol(:,1),time(1),Cq_f,Ct_f);   % Save velocities at time(1)
ddqSol(:,1) = accelerations(qSol(:,1),dqSol(:,1),...
    time(1),Cq_f,Cqt_f,Ctt_f,Cqqdq_f);                  % Save accelerations at time(1)
lambdaSol(:,1) = lagrangeMultipliers(qSol(:,1),ddqSol(:,1),time(1),Cq_f,M);                 % Save lagrange multipliers at time(1)
Qc(:,1) = reactionForces(qSol(:,1),time(1),lambdaSol(:,1),L,Cq_f);  % Save the reaction forces at time(1

% Start the simulation
for i = 2:length(time)
    % Calculate and save the configuration at the remaining time steps using Newton Raphson
    qSol(:,i) = nietraphson(qSol(:,i-1), time(i), C_f, Cq_f, tol_I, tol_step, maxI);

    % Calculate the velocities and accelerations at the current time step
    dqSol(:,i) = velocities(qSol(:,i),time(i),Cq_f,Ct_f);
    ddqSol(:,i) = accelerations(qSol(:,i),dqSol(:,i),time(i),Cq_f,Cqt_f,Ctt_f,Cqqdq_f);
    
    % Calculate the lagrange multipliers at the current time step
    lambdaSol(:,i) = lagrangeMultipliers(qSol(:,i),ddqSol(:,i),time(i),Cq_f,M);
    % Calculate the reaction forces using the lagrange multipliers
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
