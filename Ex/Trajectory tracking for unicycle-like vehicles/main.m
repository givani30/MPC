clear all
clc
close all

%% Data definition


param.T=1e-3;                        %sampletime 
param.Tf=10;                    
param.tspan=0:param.T:param.Tf;      %tspan for prediction model generation
param.x0=[1 0 pi/2];

dim.nx=3;
dim.nu=2;
dim.N=10;                             %receding horizon
dim.t=length(param.tspan)-dim.N;    

param.simultime=param.tspan(1:dim.t); %simulation horizon

weight.Q=100*diag([4 40 0.1]);
weight.R=eye(dim.nu)/100;
weight.P=zeros(dim.nx);

Xref=reftrajgen(param,dim);
Uref=refinputgen(Xref,param,dim);
% 
%% Compute the linearized prediction model and quadratic costs

LTV=computeLTV(Xref,Uref,param,dim);
predmod=predmodgen(LTV,dim);
quadcost=quadprobgen(predmod,weight,dim);


%% The discetized reference input keeps the tracking error close to 0

% f=@(t,x) unidynamics(t,x,discreteref(t,dim,param,Uref),dim);
% x=ode1(f,0:1e-5:9,param.x0');

%% Solve the MPC problem for the LTV system

% e=zeros(dim.t+1,dim.nx);
% e(1,:)=[0.5 0.5 0.5];          %start from a nonzero initial error
% uB=zeros(dim.t,dim.nu);
% for k=1:dim.t
%     uBopt=quadprog(quadcost.H(:,:,k),quadcost.h(:,:,k)*e(k,:)');
%     uB(k,:)=uBopt(1:dim.nu)';
%     e(k+1,:)=LTV.Ad(:,:,k)*e(k,:)'+LTV.Bd(:,:,k)*uB(k,:)';
% end


%%


%% Simulation for the real system
% 
f=@(t,x) unidynamics(t,x,discreteinput(x,t,dim,quadcost,param,Uref,Xref),dim);
% Simulate the system starting close to the reference trajectory
x=ode45(f,0:1e-5:9,[1+0.3 0-0.2 pi/2-0.2]);   


%% Simulation for the real system with constraints
%constraints=constraintsgen(dim);
% f=@(t,x) unidynamics(t,x,discreteinputcon(x,t,dim,quadcost,param,Uref,Xref,constraints),dim);
% % Simulate the system starting close to the reference trajectory
% x=ode1(f,0:1e-5:9,[1+0.1 0-0.05 pi/2-0.05]);   

