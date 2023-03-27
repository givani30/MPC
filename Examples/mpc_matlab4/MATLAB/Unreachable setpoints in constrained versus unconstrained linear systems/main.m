%% Unreachable setpoints in constrained versus unconstrained optimization

clear 
close all
clc
options = optimset('Display', 'off');

%% Model definition

% Define model, cost function, and bounds.
n=2;   %state dimension
p=1;   % input dimension
A = [-1 2;0 -2];
B = [1;1];

Q = [1 2;2 6];
R = 2*eye(1);
P=zeros(n);

xstar=[3;3];
ustar=1;

% (x*,u*) is NOT an equilibrium, hence the infinite horizon cost is
% infinite. 

%% Compute closest reachable setpoint

H=blkdiag(Q,R);
h=[-Q*xstar; -R*ustar];
%equality constraints, to impose that xs,us is an equilibium
Aeq=[A-eye(n) B];  
beq=zeros(n,1);

setpoint = quadprog(H,h,[],[],Aeq,beq,[],[],[],options);
xs=setpoint(1:n);
us=setpoint(n+1:end);

%% Set up the MPC

N=5; %prediction horizon
x0=[1;1];
tfin=100;  %simulation length

[T,S]=predmodgen(A,B,N,n,p); %generate prediction matrices

%% Discrete time simulation

x=zeros(n,tfin);   % Allocate state trajectory
u=zeros(p*N,tfin);    % Allocate input trajectory

x(:,1)=x0;       % Initialize state trajectory

for k=1:tfin
    [H,h]=costgen(T,S,x(:,k),Q,R,P,N,xs,us); %generate the cost matrices
    % Set final constraint
    Aeq=S(end-n+1:end,:);
    beq=xs-T(end-n+1:end,:)*x(:,k);
    u(:,k)=quadprog(H,h,[],[],Aeq,beq,[],[],[],options);
    x(:,k+1)=A*x(:,k)+B*u(1:p,k);
end

figure(1)
plot(1:tfin+1,x);
xlabel('k');
ylabel('x');
grid minor;

%% 3)

%% Compute closest reachable setpoint

H=blkdiag(Q,R);
h=[-Q*xstar; -R*ustar];

%equality constraints, to impose that xs,us is an equilibium
Aeq=[A-eye(n) B];  
beq=zeros(n,1);

%inequality constraints, for the constrained input
Ain=[zeros(1,n), ones(1,p);
     zeros(1,n), -ones(1,p)];
bin=[2;2];

setpoint = quadprog(H,h,Ain,bin,Aeq,beq,[],[],[],options);
xc=setpoint(1:n);
uc=setpoint(n+1:end);

%% Set up the MPC

N=5; %prediction horizon
x0=[1;1];
tfin=100;  %simulation length

[T,S]=predmodgen(A,B,N,n,p); %generate prediction matrices

%% Discrete time simulation

x=zeros(n,tfin);   % Allocate state trajectory
u=zeros(p*N,tfin);    % Allocate input trajectory

x(:,1)=x0;       % Initialize state trajectory

for k=1:tfin
    [H,h]=costgen(T,S,x(:,k),Q,R,P,N,xc,uc); %generate the cost matrices
%     [H,h]=costgen(T,S,x(:,k),Q,R,P,N,xstar,ustar); %generate the cost matrices
    % Set final constraint
    Aeq=S(end-n+1:end,:);
    beq=xs-T(end-n+1:end,:)*x(:,k);
    Ain=kron(eye(N),[1;-1]);
    bin=2*ones(2*N,1);
    u(:,k)=quadprog(H,h,Ain,bin,Aeq,beq,[],[],[],options);
    x(:,k+1)=A*x(:,k)+B*u(1:p,k);
end

figure(2)
plot(1:tfin+1,x);
xlabel('k');
ylabel('x');
grid minor;
