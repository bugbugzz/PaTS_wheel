function FlexibleExcavator_Final()
    % WRAPPED IN A FUNCTION TO ENSURE SUB-FUNCTIONS WORK
    close all; clc;

    %% 1. INITIALISATION & PARAMETERS (From your rigid code)
    L_in = [0.74/2; 0.74/2; 0.74; 0.25]; 
    rho = [7870; 7870; 7870; 7870];
    D = [100; 100; 100; 100]; 
    
    % Section Properties
    r_cs = (D./1000)./2;
    Area = pi .* r_cs.^2;
    Inertia = (pi/4) .* r_cs.^4; 
    E_modulus = 210e9; 

    % --- GEOMETRY SETUP ---
    thetaA_init = atan2(L_in(4), L_in(3)); % Positive angle
    thetaB_init = deg2rad(0);              % Bottom arm Horizontal
    th3_init = thetaB_init - pi/2;         % Scoop Vertical (-90 deg)

    hypot_len = sqrt(L_in(3)^2 + L_in(4)^2); 
    
    % Map to Body Lengths [L1, L2, Scoop, Arm]
    L = zeros(4,1);
    L(1) = hypot_len * 0.6; 
    L(2) = hypot_len * 0.6;
    L(4) = L_in(4);          % Scoop Height
    L(3) = L_in(3);          % Arm Length (Body 4)
    
    Len_bodies = [L(1); L(2); L(4); L(3)]; 

    %% 2. FLEXIBLE BODY SETUP
    fprintf('Calculating Flexible Modes for Body 4...\n');
    NoN = 5;               % Number of Nodes
    NoM_flex = 2;          % Number of flexible modes

    % -- Rigid Bodies 1, 2, 3 --
    M_rigid = cell(3,1);
    for i = 1:3
        m_val = rho(i) * Len_bodies(i) * Area(i);
        I_val = (1/12) * m_val * Len_bodies(i)^2;
        M_rigid{i} = diag([m_val, m_val, I_val]);
    end

    % -- Flexible Body 4 (The Arm) --
    % This function (Student File) returns reduced Mass/Stiffness matrices
    [phi_4, M_flex_4, K_flex_4] = reductionMK(NoN, Len_bodies(4), E_modulus, Inertia(4), rho(4), Area(4), NoM_flex);

    % Assemble Global Matrices (Block Diagonal)
    M = blkdiag(M_rigid{1}, M_rigid{2}, M_rigid{3}, M_flex_4);
    K = blkdiag(zeros(3), zeros(3), zeros(3), K_flex_4);

    % -- Initial Positions Estimate --
    q0_rigid = zeros(12,1);
    % Body 1
    q0_rigid(1:3) = [0.5*L(1)*cos(thetaA_init); 0.5*L(1)*sin(thetaA_init); thetaA_init];
    % Body 2
    q0_rigid(4:6) = [(hypot_len - 0.5*L(2))*cos(thetaA_init); (hypot_len - 0.5*L(2))*sin(thetaA_init); thetaA_init];
    % Body 3 (Scoop)
    q0_rigid(7:9) = [L(3)*cos(thetaB_init); 0.5*L(4)*sin(thetaB_init); th3_init];
    % Body 4 (Rigid Part)
    q0_rigid(10:12) = [0.5*L(3)*cos(thetaB_init); 0.5*L(3)*sin(thetaB_init); thetaB_init];

    % Full State: [RigidCoords; FlexibleModes]
    q0 = [q0_rigid(1:9); q0_rigid(10:12); zeros(NoM_flex, 1)];
    
    %% 3. SIMULATION SETTINGS
    A_w  = deg2rad(10);             
    A_V  = 0.05;         
    initial_len_val = hypot_len - 0.5*L(1) - 0.5*L(2);

    t_start = 0; dt = 0.005; t_stop = 8.0; 
    time_vec = t_start:dt:t_stop;

    % Gravity
    Qa_vec = zeros(length(q0),1);
    g = 9.81;
    Qa_vec(2) = -M_rigid{1}(1,1)*g;
    Qa_vec(5) = -M_rigid{2}(1,1)*g;
    Qa_vec(8) = -M_rigid{3}(1,1)*g;
    Qa_vec(11) = -M(10,10)*g; % Approx gravity on Flex Body Rigid Coords
    Qa_fun = @(t) Qa_vec;

    %% 4. SYMBOLIC CONSTRAINTS
    fprintf('Generating Equations of Motion (Symbolic)...\n');
    q = sym('q', [length(q0), 1]); 
    dq = sym('dq', [length(q0), 1]); 
    syms t
    syms w_drv V_drv dw_drv dV_drv ddw_drv ddV_drv

    x1=q(1); y1=q(2); th1=q(3); 
    x2=q(4); y2=q(5); th2=q(6);
    x3=q(7); y3=q(8); th3=q(9);
    x4=q(10); y4=q(11); th4=q(12);
    qf4 = q(13:end); 

    % -- FLEXIBLE KINEMATICS (Body 4) --
    R4 = [cos(th4), -sin(th4); sin(th4), cos(th4)];
    L4_len = Len_bodies(4);
    
    % Node 1 (Start) and Node N (End)
    idx_node1 = 1:2;
    idx_nodeN = (NoN-1)*3+1 : (NoN-1)*3+2;
    loc_node1 = [-L4_len/2; 0];
    loc_nodeN = [L4_len/2; 0];

    Phi_trans_1 = phi_4(idx_node1, 4:end); 
    Phi_trans_N = phi_4(idx_nodeN, 4:end); 

    % Global Position of Nodes = r_rigid + R * (u_local + Phi*q_flex)
    pos_Node1 = [x4;y4] + R4 * (loc_node1 + Phi_trans_1 * qf4);
    pos_NodeN = [x4;y4] + R4 * (loc_nodeN + Phi_trans_N * qf4);

    % -- CONSTRAINTS --
    % 1. Body 1 Pinned
    eq1 = x1 - 0.5*L(1)*cos(th1); 
    eq2 = y1 - 0.5*L(1)*sin(th1);
    
    % 2. Body 4 (Flexible) Node 1 Pinned to Ground
    eq3 = pos_Node1(1); 
    eq4 = pos_Node1(2);
    
    % 3. Slider
    eq5 = th1 - th2; 
    eq6 = (y2 - y1)*cos(th1) - (x2 - x1)*sin(th1);

    % 4. Connection Body 2 Tip -> Body 3 Top
    pos_Tip2_x = x2 + 0.5*L(2)*cos(th2);
    pos_Tip2_y = y2 + 0.5*L(2)*sin(th2);
    pos_Top3_x = x3 - (L(4)/2)*sin(th3);
    pos_Top3_y = y3 + (L(4)/2)*cos(th3);
    
    eq7 = pos_Tip2_x - pos_Top3_x;
    eq8 = pos_Tip2_y - pos_Top3_y;

    % 5. Connection Body 4 (Flexible) Node N -> Body 3 Bottom
    pos_Bot3_x = x3 + (L(4)/2)*sin(th3);
    pos_Bot3_y = y3 - (L(4)/2)*cos(th3);
    
    eq9  = pos_NodeN(1) - pos_Bot3_x;
    eq10 = pos_NodeN(2) - pos_Bot3_y;

    % 6. Drivers
    eq11 = th4 - (thetaB_init + w_drv);
    current_piston_len = (x2 - x1)*cos(th1) + (y2 - y1)*sin(th1);
    eq12 = current_piston_len - (initial_len_val + V_drv);

    C_sym = [eq1; eq2; eq3; eq4; eq5; eq6; eq7; eq8; eq9; eq10; eq11; eq12];

    % -- JACOBIANS --
    Cq_sym = jacobian(C_sym, q);
    Ct_sym = diff(C_sym, w_drv)*dw_drv + diff(C_sym, V_drv)*dV_drv;
    Ctt_sym = diff(C_sym, w_drv)*ddw_drv + diff(C_sym, V_drv)*ddV_drv;
    Cqqdq_sym = jacobian(Cq_sym * dq, q) * dq; 

    % Create Functions
    vars = {q, dq, w_drv, V_drv, dw_drv, dV_drv, ddw_drv, ddV_drv};
    C_f = matlabFunction(C_sym, 'vars', vars);
    Cq_f = matlabFunction(Cq_sym, 'vars', vars);
    Ct_f = matlabFunction(Ct_sym, 'vars', vars);
    Ctt_f = matlabFunction(Ctt_sym, 'vars', vars);
    Cqqdq_f = matlabFunction(Cqqdq_sym, 'vars', vars);

    %% 4.5 ASSEMBLY CORRECTION
    fprintf('Correcting Initial Assembly...\n');
    options = optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt');
    fun_assembly = @(u) C_f(u, zeros(size(u)), 0, 0, 0, 0, 0, 0);
    q0 = fsolve(fun_assembly, q0, options);

    %% 5. PREPARE DAE SOLVER
    % We define a simplified handle for PrepareDAE if strictly needed,
    % but we are using a custom dynamic loop here, so we skip PrepareDAE call 
    % and go straight to time integration using our generated matrices.

    %% 6. TIME INTEGRATION
    fprintf('Starting Dynamic Simulation...\n');
    qSol = NaN(length(q0), length(time_vec)); 
    dqSol = NaN(length(q0), length(time_vec));
    qSol(:,1) = q0; dqSol(:,1) = zeros(size(q0));

    NoC = length(C_sym);

    for i = 2:length(time_vec)
        t_now = time_vec(i-1);
        
        % Get Driver Values
        [w, V, dw, dV, ddw, ddV] = get_driver_state(t_now, A_w, A_V);
        
        % Dynamic Step with Projection
        [q_new, dq_new] = dynamic_step_projection(qSol(:,i-1), dqSol(:,i-1), t_now, dt, ...
            M, K, Qa_fun, C_f, Cq_f, Ct_f, Ctt_f, Cqqdq_f, w, V, dw, dV, ddw, ddV, NoC);
        
        qSol(:,i) = q_new;
        dqSol(:,i) = dq_new;
        
        if mod(i,100)==0, fprintf('Time: %.2f s\n', t_now); end
    end

    %% 7. PLOTTING
    figure(1); clf; hold on; axis equal; grid on;
    xlabel('X [m]'); ylabel('Y [m]'); title('Flexible Excavator');
    axis([-0.5 2.0 -1.5 1.5]); 

    % Init Plot Lines
    h_b1 = plot(0,0,'k-','LineWidth',2);
    h_b2 = plot(0,0,'b-','LineWidth',2);
    h_sc = plot(0,0,'m-','LineWidth',2);
    h_fl = plot(0,0,'g-o','LineWidth',1.5);

    fprintf('Animation starting...\n');
    for i = 1:5:length(time_vec)
        q_c = qSol(:,i);
        
        % Rigid Body Plotting
        [x,y] = get_rigid_ends(q_c(1:3), L(1));
        set(h_b1, 'XData', x, 'YData', y);
        
        [x,y] = get_rigid_ends(q_c(4:6), L(2));
        set(h_b2, 'XData', x, 'YData', y);
        
        [x,y] = get_scoop_shape(q_c(7:9), L(4), 0.2);
        set(h_sc, 'XData', x, 'YData', y);
        
        % Flexible Body Plotting
        [x,y] = get_flex_shape(q_c(10:12), q_c(13:end), L(3), phi_4, NoN);
        set(h_fl, 'XData', x, 'YData', y);
        
        drawnow;
    end
end

%% --- HELPER FUNCTIONS ---

function [w, V, dw, dV, ddw, ddV] = get_driver_state(t, A_w, A_V)
    ti = mod(t, 4.0);
    w=0; V=0; dw=0; dV=0; ddw=0; ddV=0;
    if ti <= 1.0          % Scoop
        V = A_V * (0.5 * (1 - cos(pi * ti))); 
        dV = A_V * (0.5 * pi * sin(pi * ti));
        ddV = A_V * (0.5 * pi^2 * cos(pi * ti));
    elseif ti <= 2.0      % Lift
        V = A_V; 
        w = A_w * (0.5 * (1 - cos(pi * (ti - 1.0))));
        dw = A_w * (0.5 * pi * sin(pi * (ti - 1.0)));
        ddw = A_w * (0.5 * pi^2 * cos(pi * (ti - 1.0)));
    elseif ti <= 3.0      % Drop
        V = A_V * (0.5 * (1 + cos(pi * (ti - 2.0))));
        dV = -A_V * (0.5 * pi * sin(pi * (ti - 2.0)));
        ddV = -A_V * (0.5 * pi^2 * cos(pi * (ti - 2.0)));
        w = A_w;      
    else                  % Return
        V = 0;
        w = A_w * (0.5 * (1 + cos(pi * (ti - 3.0))));
        dw = -A_w * (0.5 * pi * sin(pi * (ti - 3.0)));
        ddw = -A_w * (0.5 * pi^2 * cos(pi * (ti - 3.0)));
    end
end

function [q_next, dq_next] = dynamic_step_projection(q, dq, t, dt, M, K, Qa_fun, C_f, Cq_f, Ct_f, Ctt_f, Cqqdq_f, w, V, dw, dV, ddw, ddV, NoC)
    % 1. Evaluate Matrices
    Cq = Cq_f(q, dq, w, V, dw, dV, ddw, ddV);
    Ct = Ct_f(q, dq, w, V, dw, dV, ddw, ddV);
    Ctt = Ctt_f(q, dq, w, V, dw, dV, ddw, ddV);
    Cqqdq = Cqqdq_f(q, dq, w, V, dw, dV, ddw, ddV);
    Qa = Qa_fun(t);
    
    % 2. Solve EOM
    Damping = 0.05 * K; 
    LHS = [M, Cq'; Cq, zeros(NoC)];
    RHS_Force = Qa - K*q - Damping*dq;
    RHS_Constr = -Cqqdq - 2*0 - Ctt; 
    
    x = pinv(LHS) * [RHS_Force; RHS_Constr];
    acc = x(1:length(q));
    
    % 3. Integrate
    dq_pred = dq + acc * dt;
    q_pred  = q + dq_pred * dt;
    
    % 4. Coordinate Projection (Fixes Drift/Separation)
    q_next = q_pred;
    [w_n, V_n, ~, ~, ~, ~] = get_driver_state(t+dt, 20*pi/180, 0.2); 
    
    for k=1:10
        C_val = C_f(q_next, zeros(size(q)), w_n, V_n, 0, 0, 0, 0);
        if norm(C_val)<1e-6, break; end
        J = Cq_f(q_next, zeros(size(q)), w_n, V_n, 0, 0, 0, 0);
        q_next = q_next - pinv(J)*C_val;
    end
    dq_next = (q_next - q)/dt;
end

function [x, y] = get_rigid_ends(q, Len)
    cx = q(1); cy = q(2); th = q(3);
    x = [cx - 0.5*Len*cos(th), cx + 0.5*Len*cos(th)];
    y = [cy - 0.5*Len*sin(th), cy + 0.5*Len*sin(th)];
end

function [x, y] = get_scoop_shape(q, Len, w)
    cx = q(1); cy = q(2); th = q(3);
    pt = [cx - (Len/2)*sin(th); cy + (Len/2)*cos(th)];
    pb = [cx + (Len/2)*sin(th); cy - (Len/2)*cos(th)];
    p_tip = pb + [w*cos(th); w*sin(th)];
    x = [pt(1), pb(1), p_tip(1)];
    y = [pt(2), pb(2), p_tip(2)];
end

function [x, y] = get_flex_shape(qr, qf, Len, phi, NoN)
    cx = qr(1); cy = qr(2); th = qr(3);
    R = [cos(th) -sin(th); sin(th) cos(th)];
    dx = Len / (NoN-1);
    local_x = -Len/2 : dx : Len/2;
    x = zeros(1, NoN); y = zeros(1, NoN);
    for k = 1:NoN
        idx = (k-1)*3 + (1:2);
        def = phi(idx, 4:end) * qf;
        glob = [cx;cy] + R * ([local_x(k); 0] + def);
        x(k) = glob(1); y(k) = glob(2);
    end
end