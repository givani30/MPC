%%
clear 
clc

%% Data

%Definition of the LTI system
LTI.A=[1 2 3; 1 2 1; 1 3 1]; 
LTI.B=[0; 0; 1];
% LTI.C=[0 0 0; 0 2 1; 0 0 1];
LTI.C=eye(3);
x0=[2;3;4];

%Definition of system dimension
dim.nx=3;     %state dimension
dim.ny=3;     %output dimension
dim.nu=1;     %input dimension
dim.N=5;      %horizon

%Definition of quadratic cost function
Q=eye(dim.ny);   %weight on output
R=2;             %weight on input

%% Prediction model and cost function

[P,S]=predmodgen(LTI,dim);            %Generation of prediction model 
[H,h,const]=costgen(P,S,Q,R,dim,x0);  %Writing cost function in quadratic form

%% Solve the optimization problem at time 0: Solver YALMIP 

u = sdpvar(5,1);                % define optimization variable 5x1

% Constraints = [A*u<=b];
Constraint=[];                  %define constraints

Objective = 0.5*u'*H*u+h'*u;  %define cost function

% Set some options for YALMIP and solver
% options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);

optimize(Constraint,Objective);  %solve the problem
uopt=value(u);                   %assign the solution to uopt1


%% Compute x(1) and update cost function

x1=LTI.A*x0+LTI.B*uopt(1:dim.nu);
[H1,h1,const1]=costgen(P,S,Q,R,dim,x1);

%% Solve the optimization problem at time 1: 

%Solver CVX

cvx_begin 
    variable uopt1(dim.nu*dim.N)
    minimize(0.5*uopt1'*H1*uopt1+h1'*uopt1)
cvx_end


%Solver YALMIP

% u = sdpvar(5,1);                % define optimization variable 5x1
% 
% Constraint=[];                  %define constraints
% 
% Objective = 0.5*u'*H1*u+h1'*u;  %define cost function
% 
% % Set some options for YALMIP and solver
% % options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);
% 
% optimize(Constraint,Objective);  %solve the problem
% uopt1=value(u);                  %assign the solution to uopt1

%% Constrained optimization problem
[A,b]=constraintgen(dim);
 
%% Solve the constrained optimization problem

% Solver CVX
% 

cvx_begin 
    variable uopt2(dim.nu*dim.N)
    minimize(0.5*uopt2'*H*uopt2+h'*uopt2)
    subject to
    A*uopt2<=b;
cvx_end

% %YALMIP
%     u = sdpvar(5,1);                % define optimization variable 5x1
% 
%     Constraint=[A*u<=b];                  %define constraints
% 
%     Objective = 0.5*u'*H*u+h'*u;  %define cost function
% 
%     % Set some options for YALMIP and solver
%     % options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);
% 
%     optimize(Constraint,Objective);  %solve the problem
%     uopt2=value(u);                  %assign the solution to uopt1

% %Gurobi
%     MPC=gurobimodelgen(H,h,A,b);
%     solution = gurobi(MPC);   % Seeking for a solution...
%     uopt2 = solution.x;