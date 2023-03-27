%%
clear all
close all
clc

%% Data & Initialization

% Definition of the LTI system
LTI.A = [2 1;0 2];
LTI.B = eye(2);

x0 = [.5;.5];

% Definition of system dimension
dim.nx = size(LTI.A,2);     % state dimension
dim.nu = size(LTI.B,2);     % input dimension

dim.N = 3;      % prediction horizon

% State constraint
A = [1 0];
b = 5;

% Input constraint
C = blkdiag(eye(2),-eye(2));
d = ones(dim.nu*2);

%Definition of quadratic cost function
Q = eye(dim.nx);   %weight on output
R = eye(dim.nu);             %weight on input

% Generation of prediction model
[P_state,S_state]=predmodgen_state(LTI,dim);    

% Set some options for YALMIP and solver
options = sdpsettings('verbose',0,'solver','quadprog');

%% a)
% Receding horizon implementation for the unconstrained control problem
T=30;
x_0 = x0;
x(:,1) = x0;

alpha = [.2 .6 1 5];

figure(1), xlabel('$x_1$','Interpreter','latex'), ylabel('$x_2$','Interpreter','latex'), grid on, hold on;
figure(2), xlabel('k'), ylabel('u'), grid on, hold on;

for i=1:length(alpha) 
    for k=1:T

        % Write the cost function in quadratic form
        [H,h,const]=costgen(P_state(1:end-dim.nx,:),S_state(1:end-dim.nx,:),alpha(i)*Q,R,dim,x_0); 

        % Solve the constrained optimization problem (with YALMIP)
        u_uncon = sdpvar(dim.nu*dim.N,1);                % define optimization variable

        Constraint=[];                  %define constraints

        Objective = 0.5*u_uncon'*H*u_uncon+h'*u_uncon;  %define cost function

        optimize(Constraint,Objective,options);  %solve the problem
        u_uncon=value(u_uncon);                  %assign the solution to uopt

        % Select the first input only
        u_rec(:,k) = u_uncon(1:dim.nu);

        % Compute the state/output evolution
        x(:,k+1)=LTI.A*x_0 + LTI.B*u_rec(:,k);

        % Update initial state for the next iteration
        x_0=x(:,k+1);

        clear u_uncon
    end
    
    figure(1),
    plot(x(1,:), x(2,:), '-o');
    
    figure(2)
    plot(0:T-1, u_rec, '-o');
    
    clear u_rec x
    
    x_0 = x0;
    x(:,1) = x0;
end

%% b)
% Receding horizon implementation for the constrained control problem
alpha = .2;

T=30;
x_0 = x0;
x(:,1) = x0;

% Set the constraint matrix
A_aux = S_state(1:2:end,:);
A_aux = A_aux(1:end-1,:);

P_aux = P_state(1:2:end,:);
P_aux = P_aux(1:end-1,:);

A = [A_aux;...
    eye(dim.nu*dim.N);...
    -eye(dim.nu*dim.N)];

for k=1:T
    
    % Set the RHS terms of the constraint
    b=[5*ones((dim.nx-1)*dim.N,1) - P_aux*x_0;...
        ones(2*dim.nu*(dim.N),1)];
    
    % Write the cost function in quadratic form
    [H,h,const]=costgen(P_state(1:end-dim.nx,:),S_state(1:end-dim.nx,:),alpha*Q,R,dim,x_0); 

    % Solve the constrained optimization problem (with YALMIP)
    u_con = sdpvar(dim.nu*dim.N,1);                % define optimization variable

    Constraint = [A*u_con<=b];                  %define constraints

    Objective = 0.5*u_con'*H*u_con+h'*u_con;  %define cost function

    optimize(Constraint,Objective,options);  %solve the problem
    u_con = value(u_con);                  %assign the solution to uopt

    
    % Select the first input only
    u_rec(:,k) = u_con(1:dim.nu);

    % Compute the state/output evolution
    x(:,k+1) = LTI.A*x_0 + LTI.B*u_rec(:,k);
    
    % Update initial state for the next iteration
    x_0 = x(:,k+1);
    
    clear u_con
end

% Plots
figure,
plot(x(1,:), x(2,:), '-o'),
xlabel('$x_1$','Interpreter','latex'), ylabel('$x_2$','Interpreter','latex'), grid on;

figure,
plot(0:T-1, u_rec, '-o'),
xlabel('k'), ylabel('u'), grid on;

%% c)
% Receding horizon implementation for the constrained control problem
alpha = .2;

T=30;
x_0 = x0;
x(:,1) = x0;

% Set the constraint matrix
A_aux = S_state(1:2:end,:);
A_aux = A_aux(1:end-1,:);
A_eq = S_state(end-1:end,:);

P_aux = P_state(1:2:end,:);
P_aux = P_aux(1:end-1,:);
P_eq = P_state(end-1:end,:);

A = [A_aux;...
    eye(dim.nu*dim.N);...
    -eye(dim.nu*dim.N)];

for k=1:T
    
    % Set the RHS terms of the constraint
    b=[5*ones((dim.nx-1)*dim.N,1) - P_aux*x_0;...
        ones(2*dim.nu*(dim.N),1)];
    
    b_eq = [-P_eq*x_0];
    
    % Write the cost function in quadratic form
    [H,h,const]=costgen(P_state(1:end-dim.nx,:),S_state(1:end-dim.nx,:),alpha*Q,R,dim,x_0); 

    % Solve the constrained optimization problem (with YALMIP)
    u_con = sdpvar(dim.nu*dim.N,1);                % define optimization variable

    Constraint = [A*u_con<=b;
                  A_eq*u_con==b_eq];                  %define constraints

    Objective = 0.5*u_con'*H*u_con+h'*u_con;  %define cost function

    optimize(Constraint,Objective,options);  %solve the problem
    u_con = value(u_con);                  %assign the solution to uopt

    
    % Select the first input only
    u_rec(:,k) = u_con(1:dim.nu);

    % Compute the state/output evolution
    x(:,k+1) = LTI.A*x_0 + LTI.B*u_rec(:,k);
    
    % Update initial state for the next iteration
    x_0 = x(:,k+1);
    
    clear u_con
end

% Plots
figure,
plot(x(1,:), x(2,:), '-o'),
xlabel('$x_1$','Interpreter','latex'), ylabel('$x_2$','Interpreter','latex'), grid on;

figure,
plot(0:T-1, u_rec, '-o'),
xlabel('k'), ylabel('u'), grid on;

%%
% Estimate of the region of attraction with the algorithm synthesized in
% exercise "Computing the projection set"
P_aux = P_state(1:2:end,:);
P_aux = P_aux(1:end-1,:);

% We shall consider the equality constraint as well
P_eq = P_state(end-1:end,:);

A_aux = S_state(1:2:end,:);
A_aux = A_aux(1:end-1,:);
A_eq = S_state(end-1:end,:);

G_x =[P_aux; P_eq; -P_eq];
H_x = [A_aux; A_eq; -A_eq];
  
Psi_x = [b*ones(dim.N,1);
         zeros(2*dim.nx,1)];

% Define input constraints
H_u = [eye(dim.nu*dim.N);...
    -eye(dim.nu*dim.N)];
Psi_u = ones(2*dim.nu*dim.N,1);

G = [G_x; zeros(size(H_u,1),dim.nx)];
H = [H_x; H_u];
Psi = -[Psi_x; Psi_u];

% Algorithm implementation
G_i = [G H(:,1:end-1)];
H_i = H(:,end);
Psi_i = Psi;

for i = dim.N*dim.nu-1:-1:0

    [P_i, gamma_i] = single_input(G_i,H_i,Psi_i);
    
    G_i = P_i(:,1:end-1);
    H_i = P_i(:,end);
    Psi_i = gamma_i;
    
end

P = P_i;
gamma = gamma_i;

% Post-processing
% Eliminate zero-rows
nonzero_index = [];
for i = 1:size(P,1)
    
    if all(P(i,:)==0) == 0
        nonzero_index = [nonzero_index i];
    end
    
end

P = P(nonzero_index,:);
gamma = gamma(nonzero_index);

% Eliminate repeated rows
norep = unique([P gamma], 'rows');

P = norep(:,1:end-1);
gamma = norep(:,end);

% Plot region
x_1 = -3:.01:3;
x_2 = -3:.01:3;

[X_1, X_2] = meshgrid(x_1, x_2);

for i = 1:size(P,1)
   
    map(:,:,i) = (P(i,2).*X_2 <= -P(i,1).*X_1 - gamma(i));
    
end

figure,
contourf(X_1, X_2, min(double(map), [], 3), [1 1]);
xlabel('$x_1$','Interpreter','latex'), ylabel('$x_2$','Interpreter','latex'), grid on;
xlim([-2 2]), ylim([-2 2]);









