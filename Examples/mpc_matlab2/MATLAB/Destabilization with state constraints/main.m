%%
clear all
close all
clc

%% Data & Initialization

%Definition of the LTI system
LTI.A=[4/3 -2/3; 1 0]; 
LTI.B=[1; 0];
LTI.C=[-2/3 1];
x0=[3;3];

%Definition of system dimension
dim.nx=2;     %state dimension
dim.ny=1;     %output dimension
dim.nu=1;     %input dimension
dim.N=5;      %horizon

%Show the properties of such system
z=tf('z');
[num,den]=ss2tf(LTI.A,LTI.B,LTI.C,0);
G=tf(num,den,-1);
pzmap(G);    % Note that the zero is outside the unit circle

%Definition of quadratic cost function
Q=eye(dim.nx);   %weight on output
R=1;             %weight on input

% Generation of prediction model (both for state and output)
[P_state,S_state]=predmodgen_state(LTI,dim);            
[P_output,S_output]=predmodgen_output(LTI,dim);

%% a)
% Receding horizon implementation
T=30;
x_0 = x0;
x(:,1)=x0;
y(1)=LTI.C*x0;

for k=1:T
    
    % Write the cost function in quadratic form
    [H,h,const]=costgen(P_state(dim.nx+1:end-dim.nx,:),S_state(dim.nx+1:end-dim.nx,:),Q,R,dim,x_0); 
    
%     % Solve the optimization problem at time k (with CVX)
%     cvx_begin 
%         variable u_uncon(dim.nu*dim.N)
%         minimize(0.5*u_uncon'*H*u_uncon+h'*u_uncon)
%     cvx_end
    
    % Solve the constrained optimization problem (with YALMIP)
    u_uncon = sdpvar(dim.nu*dim.N,1);                % define optimization variable

    Constraint=[];                  %define constraints

    Objective = 0.5*u_uncon'*H*u_uncon+h'*u_uncon;  %define cost function

    optimize(Constraint,Objective);  %solve the problem
    u_uncon=value(u_uncon);                  %assign the solution to uopt
    
    % Select the first input only
    u_rec(k)=u_uncon(1);

    % Compute the state/output evolution
    x(:,k+1)=LTI.A*x_0 + LTI.B*u_rec(k);
    y(k+1)=LTI.C*x(:,k+1);
    
    % Update initial state for the next iteration
    x_0=x(:,k+1);
    
    clear u_uncon
end

% Plots
figure,
plot(0:T, y, '-o'),
xlabel('k'), ylabel('y'), grid on;

figure,
plot(x(1,:), x(2,:), '-o'),
xlabel('$x_1$','Interpreter','latex'), ylabel('$x_2$','Interpreter','latex'), grid on;

figure,
plot(0:T-1, u_rec, '-o'),
xlabel('k'), ylabel('u'), grid on;

%% b)
% Receding horizon implementation
T=30;
x_0 = x0;
x(:,1)=x0;
y(1)=LTI.C*x0;

% Set the constraint matrix
A=[S_output(dim.ny+1:end-dim.ny,:); -S_output(dim.ny+1:end-dim.ny,:)];

for k=1:T
    
    % Set the RHS terms of the constraint
    b=[.5*ones(dim.N-1,1) - P_output(dim.ny+1:end-dim.ny,:)*x_0;...
        .5*ones(dim.N-1,1) + P_output(dim.ny+1:end-dim.ny,:)*x_0]; 
    
    % Write the cost function in quadratic form
    [H,h,const]=costgen(P_state(dim.nx+1:end-dim.nx,:),S_state(dim.nx+1:end-dim.nx,:),Q,R,dim,x_0); 
    
    % Solve the constrained optimization problem (with CVX)
%     cvx_begin 
%         variable u_con(dim.nu*dim.N)
%         minimize(0.5*u_con'*H*u_con+h'*u_con)
%         subject to
%         A*u_con<=b;
%     cvx_end

    % Solve the constrained optimization problem (with YALMIP)
    u_con = sdpvar(dim.nu*dim.N,1);                % define optimization variable

    Constraint=[A*u_con<=b];                  %define constraints

    Objective = 0.5*u_con'*H*u_con+h'*u_con;  %define cost function

    % Set some options for YALMIP and solver
    % options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);

    optimize(Constraint,Objective);  %solve the problem
    u_con=value(u_con);                  %assign the solution to uopt

    
    % Select the first input only
    u_rec(k)=u_con(1);

    % Compute the state/output evolution
    x(:,k+1)=LTI.A*x_0 + LTI.B*u_rec(k);
    y(k+1)=LTI.C*x(:,k+1);
    
    % Update initial state for the next iteration
    x_0=x(:,k+1);
    
    clear u_con
end

% Plots
figure,
plot(0:T, y, '-o'),
xlabel('k'), ylabel('y'), grid on;

figure,
plot(x(1,:), x(2,:), '-o'),
xlabel('$x_1$','Interpreter','latex'), ylabel('$x_2$','Interpreter','latex'), grid on;

figure,
plot(0:T-1, u_rec, '-o'),
xlabel('k'), ylabel('u'), grid on;

%% c)
% Set the constraint matrix
A=[S_output(dim.ny+1:end-dim.ny,:); -S_output(dim.ny+1:end-dim.ny,:)];
epsilon=[1e-3 .1 1 5 10 50];

% Receding horizon implementation
T=30;

figure(1), xlabel('k'), ylabel('y'), grid on, hold on;
figure(2), xlabel('$x_1$','Interpreter','latex'), ylabel('$x_2$','Interpreter','latex'), grid on, hold on;
figure(3), xlabel('k'), ylabel('u'), grid on, hold on;

for i=1:length(epsilon)   
    x_0 = x0;
    x(:,1)=x0;
    y(1)=LTI.C*x0;
    
    for k=1:T
        % Set the RHS terms of the constraint
        b=[(1+epsilon(i))*ones(dim.N-1,1) - P_output(dim.ny+1:end-dim.ny,:)*x_0;...
            (1+epsilon(i))*ones(dim.N-1,1) + P_output(dim.ny+1:end-dim.ny,:)*x_0]; 

        % Write the cost function in quadratic form
        [H,h,const]=costgen(P_state(dim.nx+1:end-dim.nx,:),S_state(dim.nx+1:end-dim.nx,:),Q,R,dim,x_0); 

        % Solve the constrained optimization problem (with CVX)
    %     cvx_begin 
    %         variable u_con(dim.nu*dim.N)
    %         minimize(0.5*u_con'*H*u_con+h'*u_con)
    %         subject to
    %         A*u_con<=b;
    %     cvx_end

        % Solve the constrained optimization problem (with YALMIP)
        u_con = sdpvar(dim.nu*dim.N,1);                % define optimization variable

        Constraint=[A*u_con<=b];                  %define constraints

        Objective = 0.5*u_con'*H*u_con+h'*u_con;  %define cost function

        % Set some options for YALMIP and solver
        % options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);

        optimize(Constraint,Objective);  %solve the problem
        u_con=value(u_con);                  %assign the solution to uopt


        % Select the first input only
        u_rec(k)=u_con(1);

        % Compute the state/output evolution
        x(:,k+1)=LTI.A*x_0 + LTI.B*u_rec(k);
        y(k+1)=LTI.C*x(:,k+1);

        % Update initial state for the next iteration
        x_0=x(:,k+1);
        
        clear u_con
    end
    % Plot
%     figure,
    figure(1),
    plot(0:T, y, '-o');
    
    figure(2),
    plot(x(1,:), x(2,:), '-o');
    
    figure(3)
    plot(0:T-1, u_rec, '-o');
    
    clear u_rec x y
end

%% d)
% Receding horizon implementation
T=30;
x_0 = x0;
x(:,1)=x0;
y(1)=LTI.C*x0;

% Set the constraint matrices (inequality and equality)
A=[S_output(dim.ny+1:end-dim.ny,:); -S_output(dim.ny+1:end-dim.ny,:)];
A_eq=S_state(end-dim.nx+1:end,:);

for k=1:T
    
    % Set the RHS terms of the constraint (inequality and equality)
    b=[.5*ones(dim.N-1,1) - P_output(dim.ny+1:end-dim.ny,:)*x_0;...
        .5*ones(dim.N-1,1) + P_output(dim.ny+1:end-dim.ny,:)*x_0];
    b_eq=-P_state(end-dim.nx+1:end,:)*x_0;
    
    % Write the cost function in quadratic form
    [H,h,const]=costgen(P_state(dim.nx+1:end-dim.nx,:),S_state(dim.nx+1:end-dim.nx,:),Q,R,dim,x_0); 
    
    % Solve the constrained optimization problem (with CVX)
%     cvx_begin 
%         variable u_con(dim.nu*dim.N)
%         minimize(0.5*u_con'*H*u_con+h'*u_con)
%         subject to
%         A*u_con<=b;
%         A_eq*u_con==b_eq;
%     cvx_end

    % Solve the constrained optimization problem (with YALMIP)
    u_con = sdpvar(dim.nu*dim.N,1);                % define optimization variable

    Constraint=[A*u_con<=b,...
                A_eq*u_con==b_eq];                  %define constraints

    Objective = 0.5*u_con'*H*u_con+h'*u_con;  %define cost function

    % Set some options for YALMIP and solver
    % options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);

    optimize(Constraint,Objective);  %solve the problem
    u_con=value(u_con);                  %assign the solution to uopt

    
    % Select the first input only
    u_rec(k)=u_con(1);

    % Compute the state/output evolution
    x(:,k+1)=LTI.A*x_0 + LTI.B*u_rec(k);
    y(k+1)=LTI.C*x(:,k+1);
    
    % Update initial state for the next iteration
    x_0=x(:,k+1);
    
    clear u_con
end

% Plots
figure,
plot(0:T, y, '-o'),
xlabel('k'), ylabel('y'), grid on;

figure,
plot(x(1,:), x(2,:), '-o'),
xlabel('$x_1$','Interpreter','latex'), ylabel('$x_2$','Interpreter','latex'), grid on;

figure,
plot(0:T-1, u_rec, '-o'),
xlabel('k'), ylabel('u'), grid on;
