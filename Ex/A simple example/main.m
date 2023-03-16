%%
clear
close all
clc

%% Data & Initialization

%Definition of the LTI system
LTI.A=[0 0; 1 0]; 
LTI.B=[1; 0];
LTI.x0=[3;3];

%Definition of system dimension
dim.nx=2;     %state dimension
dim.nu=1;     %input dimension
dim.N=2;      %horizon

%Definition of quadratic cost function
weight.Q=[1 2; 2 6];   %weight on output
weight.R=0;            %weight on input

T=30;                  %simulation horizon



% Generation of prediction model 
predmod=predmodgen(LTI,dim);            
[H,h]=costgen(predmod,weight,dim);


%% a)
x=zeros(dim.nx,T+1);
u_rec=zeros(dim.nu,T);
x(:,1)=LTI.x0;

% Receding horizon implementation
for k=1:T
    
    x_0=x(:,k);
    
    % Solve the unconstrained optimization problem (with YALMIP)
    u_uncon = sdpvar(dim.nu*dim.N,1);                        %define optimization variable
    Constraint=[];                                           %define constraints
    Objective = 0.5*u_uncon'*H*u_uncon+(h*x_0)'*u_uncon;     %define cost function
    optimize(Constraint,Objective);                          %solve the problem
    u_uncon=value(u_uncon);                                  %assign the solution
    
    % Select the first input only
    u_rec(k)=u_uncon(1);

    % Compute the state/output evolution
    x(:,k+1)=LTI.A*x_0 + LTI.B*u_rec(k);
    clear u_uncon
    
end

% Plots
figure,
plot(0:T, x),
xlabel('k'), ylabel('y'), grid on;
legend('x_1','x_2');


% By explicitely solving the optimization problem, you can write the 
% closed loops dynamics are x^+=[-2 0; 1 0]x, clearly unstable 

%% b) 

constraints=constraintgen(predmod,dim);
% 
x=zeros(dim.nx,T+1);
u_rec=zeros(dim.nu,T);
x(:,1)=LTI.x0;



for k=1:T
    
    x_0=x(:,k);  
    % Solve the constrained optimization problem (with YALMIP)
    u_con = sdpvar(dim.nu*dim.N,1);                             %define optimization variable
    Constraint=[constraints.Ae*u_con==-constraints.be*x_0];     %define constraints
    Objective = 0.5*u_con'*H*u_con+(h*x_0)'*u_con;              %define cost function
    optimize(Constraint,Objective);                             %solve the problem
    u_con=value(u_con);                                         %assign the solution
    
    % Select the first input only
    u_rec(k)=u_con(1);

    % Compute the state/output evolution
    x(:,k+1)=LTI.A*x_0 + LTI.B*u_rec(k);
    clear u_uncon
end


% Plots
figure,
plot(0:T, x),
xlabel('k'), ylabel('y'), grid on;
legend('x_1','x_2');

%% c) 
dim.N=3;
predmod=predmodgen(LTI,dim);            
[H,h]=costgen(predmod,weight,dim);


% Receding horizon implementation
x=zeros(dim.nx,T+1);
u_rec=zeros(dim.nu,T);
x(:,1)=LTI.x0;
constraints=constraintgen(predmod,dim);


for k=1:T
    
    x_0=x(:,k);
    
    % Solve the unconstrained optimization problem (with YALMIP)
    u_uncon = sdpvar(dim.nu*dim.N,1);                        %define optimization variable
    Constraint=[];                             %define constraints
    Objective = 0.5*u_uncon'*H*u_uncon+(h*x_0)'*u_uncon;     %define cost function
    optimize(Constraint,Objective);                          %solve the problem
    u_uncon=value(u_uncon);                                  %assign the solution
    
    % Select the first input only
    u_rec(k)=u_uncon(1);

    % Compute the state/output evolution
    x(:,k+1)=LTI.A*x_0 + LTI.B*u_rec(k);
    clear u_uncon
    
end

% Plots
figure,
plot(0:T, x),
xlabel('k'), ylabel('y'), grid on;
legend('x_1','x_2');