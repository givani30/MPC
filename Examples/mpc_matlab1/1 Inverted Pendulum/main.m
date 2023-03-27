%% Model definition

% Put all the parameters in a structure array
param.g=9.81; % Gravity acceleration [m/s^2]
param.l=0.79; % Length of the pendulum [m]
param.m=0.24; % Mass of the pendulum [kg]

% Define initial conditions
theta0=0.5;            % Initial angular displacement from vertical position [rad]
omega0=0;              % Initial angular velocity [rad/s] 
x0=[theta0; omega0];   

tfin=10;
tspan=0:1e-2:tfin;       % To obtain solution at specific times

%% Continuous time simulations

%Simulation without controller
u=@(t,x) 0; % Define the input, as a function of time and/or state; in this
            % case, u is the zero function
f=@(t,x)dynamics(t,x,u,param); % ODE definition
[t,x]=ode45(f,tspan,x0);       % Solving ODE


% Simulation with proportional control
up=@(t,x) -3*x(1);               % Proportional control definition
fp=@(t,x)dynamics(t,x,up,param); % ODE definition
[t,xp]=ode45(fp,tspan,x0);       % Solving ODE

% Simulation with PD control
upd=@(t,x) -3*x(1)-x(2);           % PD control definition
fpd=@(t,x)dynamics(t,x,upd,param); % ODE definition
[t,xpd]=ode45(fpd,tspan,x0);       % Solving ODE

%% Discrete time simulations (Euler)

%% Fixed-step simulations (Euler)

h=1e-3;                 % Define the time step
% h=1;
td=0:h:tfin;            % Define time span
u=@(t,x) 0;             % Zero input

% a) By Hand
xd=zeros(length(td),2); % Allocate state variable
xd(1,:)=x0;             % Initial conditions


for i=1:length(td)-1
    xd(i+1,:)=xd(i,:)+h*dynamics(td(i),xd(i,:),u,param)'; %Euler method
end

% % b)Alternatively
% f=@(t,x)dynamics(t,x,u,param);
% xd=ode1(f,td,x0);



%% Plots

% Plot angular quantities

figure(1) %CT

subplot(2,1,1)
plot(t,x(:,1));
xlabel('time [s]');
ylabel('\theta [rad]');
grid minor;
title('CT Simulation');

subplot(2,1,2)
plot(t,x(:,2));
xlabel('time [s]');
ylabel('\omega [rad/s]');
grid minor;


figure(2) %DT

subplot(2,1,1)
plot(td,xd(:,1));
xlabel('time [s]');
ylabel('\theta [rad]');
grid minor;
title('DT Simulation');

subplot(2,1,2)
plot(td,xd(:,2));
xlabel('time [s]');
ylabel('\omega [rad/s]');
grid minor;

% % Cartesian trajectory animation
% cartpos=param.l*[sin(xpd(:,1)) cos(xpd(:,1))]; % Compute cartesian trajectory
% figure(3)
% h=plot([0 cartpos(1,1)],[0 cartpos(1,2)],'-o','MarkerIndices',2,'MarkerSize',30,'MarkerFaceColor','red','LineWidth',5);
% hold on;
% plot(0,0,'o','MarkerSize',15,'MarkerEdgeColor','b','MarkerFaceColor','y');
% axis(1.5*param.l*[-1 1 -1 1]);
% axis square;
% grid on;
% 
% for i=2:length(t)
%     set(h,'XData',[0 cartpos(i,1)],'YData',[0 cartpos(i,2)]);
%     title(['t= ',num2str(t(i),3),' [s]']);
%     drawnow
%     pause(1e-2)
% end