clear
clc

%% System definition

% Define system matrices
A=[0 1; -1 -1]; 
B=[0; 1];
C=eye(2);
D=zeros(2,1);

x0=[2 1]';            % Initial state
% x0=randn(2,1); 

Ts=1;                 % Define sampling time

sysCT=ss(A,B,C,D)     % Define a continuous time LTI system

%% Compute Discretized System

%Exact discretization
Ad=expm(A*Ts);
Bd=A\(Ad-eye(2))*B;     % Attention! ONly if A is invertible
Cd=C;
Dd=D;

% Alternatively...
% sysDT=c2d(sysCT,Ts);  % Compute the discretized LTI system
%                       % Attention! Different from ss(A,B,C,D,1)
% Ad=sysDT.a;           
% Bd=sysDT.b;      

td=0:30;       % time span

%% LQR Control

Q=[2 0; 0 1];   % State weigth
R=2;            % input weigth


[X,L,G]=dare(Ad,Bd,Q,R);  
% Compute the unique solution X to the discrete time ARE; the function also
% returns the vector of eigenvalues of the closed loop system L, and the
% gain matrix G
% You can use lqr(sysDT,Q,R) equivalently

K=(R+Bd'*X*Bd)\Bd'*X*Ad;  % Optimal gain. Check that K=G

xd=zeros(2,length(td));   % Allocate state trajectory
u=zeros(1,size(td,2));    % Allocate input trajectory

xd(:,1)=x0;               % Initialize state trajectory

%% Discrete time simulation
for k=td(2:end)
    u(k)=-K*xd(:,k);
    xd(:,k+1)=Ad*xd(:,k)+Bd*u(k);
end

%% Plots
    
figure(1)

subplot(2,1,1)
plot(td,xd(1,:));
xlabel('k');
ylabel('x_1');
grid minor;

subplot(2,1,2)
plot(td,xd(2,:));
xlabel('k');
ylabel('x_2');
grid minor;