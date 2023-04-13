%%
clear all
close all
clc

%% Data & Initialization

%Definition of the LTI system
LTI.A=[0.1 0;0 2]; 
LTI.B=[-1; 1];
LTI.x0=[100; 0.4];


%Definition of system dimension
dim.nx=2;     %state dimension
dim.nu=1;     %input dimension
dim.N=2;      %horizon

%Definition of quadratic cost function
weight.Q=[100 0; 0 0.1];               %weight on output
weight.R=0.1*eye(dim.nu);            %weight on input
weight.P=dare(LTI.A,LTI.B,weight.Q,weight.R);

T=30;                  %simulation horizon

%% 1)
% Generation of prediction model 
predmod=predmodgen(LTI,dim);            
[H,h]=costgen(predmod,weight,dim);

% % Receding horizon implementation
x=zeros(dim.nx,T+1);
u_rec=zeros(dim.nu,T);
x(:,1)=LTI.x0;

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
% 

%% 2)
% % Generation of prediction model 
% predmod=predmodgen(LTI,dim);            
% [H,h]=costgen(predmod,weight,dim);
% 
% % % Receding horizon implementation
% x=zeros(dim.nx,T+1);
% u_rec=zeros(dim.nu,T);
% x(:,1)=LTI.x0;
% 
% for k=1:T
%     
%     x_0=x(:,k);
%     
%     % Solve the unconstrained optimization problem (with YALMIP)
%     u_con = sdpvar(dim.nu*dim.N,1);                          %define optimization variable
%     inputc.A=[eye(dim.N); -eye(dim.N)];
%     inputc.b=[0.6*ones(2*dim.N,1)];                                                 
%     Constraint=[inputc.A*u_con<=inputc.b];                   %define constraints                                            
%     Objective = 0.5*u_con'*H*u_con+(h*x_0)'*u_con;           %define cost function
%     optimize(Constraint,Objective);                          %solve the problem
%     u_con=value(u_con);                                      %assign the solution
%     
%     % Select the first input only
%     u_rec(k)=u_con(1);
% 
%     % Compute the state/output evolution
%     x(:,k+1)=LTI.A*x_0 + LTI.B*u_rec(k);
%     clear u_uncon
%     
% end
% 
%   

%% Plots
figure,
plot(0:T, x),
xlabel('k'), ylabel('y'), grid on;
legend('x_1','x_2');                                                             %assign the solution