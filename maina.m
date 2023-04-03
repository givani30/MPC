%% 
% Define system params
close all
clear all 
clc
m= 1000; %kg, mass of the vehicle
I=1500; %kgm^2, moment of inertia of the vehicle
a=1.5; %m, distance from the center of mass to the front axle
b=1.5; %m, distance from the center of mass to the rear axle
c=1; % The distance from the center of mass to the left/right side of the tires (y axis)
parameters=[m;I;a;b;c];
%% 
% Define x0 equilibrium of with speed of 50 km/h$$\xi =\left\lbrack \begin{array}{cccccc}\dot{x} 
% & \dot{y} & \dot{\psi} & \psi & Y & X\end{array}\right\rbrack =\left\lbrack 
% \begin{array}{cccccc}\frac{50}{3\ldotp 6} & 0 & 0 & 0 & 0 & 0\end{array}\right\rbrack$$
% 
% $$u=\left\lbrack \begin{array}{ccccc}\delta  & F_{f,l}  & F_{f,r}  & F_{r,l}  
% & F_{r,r} \end{array}\right\rbrack$$
V=20;
eps_0=[0;0;0;V];
lanewidth=3.5;
lanes=3;
%% 
% MPC

%Sampling time of T_s=0.05
Ts=5e-2;
u_0=zeros(2,1);
[dsys,U,Y,X,DX]=(discreteSS(eps_0,u_0,parameters,Ts));
mpcobj=mpc(dsys);
%% 
% STEPS:
%% 
% # Define prediction horizon

mpcobj.PredictionHorizon = 60; %2 sec
mpcobj.ControlHorizon=2; %0.1 Sec
%% 
% # Constraints on Control (Max Throttle, Max throttle rate of change. Max steering 
% angle,)
mpcobj.ManipulatedVariables(2).RateMin = -0.2*Ts;
mpcobj.ManipulatedVariables(2).RateMax = 0.2*Ts;
mpcobj.ManipulatedVariables(1).RateMin = -pi/30*Ts;
mpcobj.ManipulatedVariables(1).RateMax = pi/30*Ts;

mpcobj.ManipulatedVariables(1).ScaleFactor = 0.2;
mpcobj.ManipulatedVariables(2).ScaleFactor = 2;
%% ;

% # Weights on output vars
mpcobj.Weights.OutputVariables=[0 30 0 1]; %Weight on x_dot,y_dot,psi and y
% # Nominal operating point
mpcobj.Model.Nominal.X=X;
mpcobj.Model.Nominal.U=U;
mpcobj.Model.Nominal.DX=DX;
mpcobj.Model.Nominal.Y=Y;
%%
% # Define the constraints on the output variables
%Type of constraints: E*u+F*y<=G

%Create obstacle
obstacle=createObstacle();
obstacle=ObstacleGeometry(obstacle);
[E,F,G]=baseConstraints(lanewidth,lanes);
setconstraint(mpcobj, E,F,G,[1;1;0.1]);

% %Terminal constraint
% Y_term=struct('Weight',[0 5 0 0 30 0],'Min',[-10 -10 -10 -10 -lanes*lanewidth/2 0],'Max',[10 10 10 10 lanes*lanewidth/2 inf]);
% U_term=struct('Max',[max_angle maxT maxT maxT maxT]);
% setterminal(mpcobj,Y_term,U_term,mpcobj.PredictionHorizon)
% %Simulate the system
refSpeed=[V;0;0;0];

%Initial conditions
x=eps_0;
u=u_0;
y=dsys.C*eps_0;
egostates=mpcstate(mpcobj);

T=0:Ts:10;

%Vars to store simulation data
states=zeros(length(x),length(T));
inputs=zeros(length(u),length(T));
detected=zeros(1,length(T));
slopes=zeros(1,length(T));
intercepts=zeros(1,length(T));
% Simulate the system
review(mpcobj)
for i=1:length(T)
    %Update plant states
    [newsys,U,Y,X,DX]=(discreteSS(x,u,parameters,Ts));
    %Detection logic
%     if ObstacleDetected(x,obstacle):
%         [E,F,G]=updateConstraints(x,obstacle);
%     else:
%         [E,F,G]=baseConstraints()
%     end
        
    %Update constraints
    %Update nominal operating point
    newNominal=struct('X',X,'U',U,'DX',DX,'Y',Y);
    measurements=newsys.C*x+newsys.D*u;
    opt=mpcmoveopt;
    %ADD updated constraints here
    detect=ObstacleDetect(x,obstacle);
    detected(i)=detect;
%     detect=false;
    [E,F,G,slopes(i),intercepts(i)]=updateConstraints(x,obstacle,detect,lanewidth,lanes);
    opt.CustomConstraint=struct('E',E,'F',F,'G',G);
%Update ref speed
    refSpeed=[0;0; 0; V];
    %Get the optimal control action
    [u]=mpcmoveAdaptive(mpcobj, egostates, newsys, newNominal, measurements, refSpeed, [],opt);
    %Time update of the system
    x=egostates.Plant;
    %Save the results
    states(:,i)=x;
    inputs(:,i)=u;
end
% % %Simulate the system
% for i=1:length(T)
%     
%     %Get the optimal control action
%     [u]=mpcmove(mpcobj,egostates,[],refSpeed);
%     %Simulate the system
% 
%     %Save the results
%     states(:,i)=x;
%     inputs(:,i)=u;
% end
%% 
% Plot results
figure
hold on
plot(states(1,:),states(2,:))
% 
% % Define obstacle vertices
% obstacle_vertices = [obstacle.fl; obstacle.fr; obstacle.rr; obstacle.rl];
% 
% % Plot obstacle
% rectangle('Faces', [1 2 3 4], 'Vertices', obstacle_vertices, 'FaceColor', 'red')
rectangle(Position=[obstacle.rrX,obstacle.rrY,obstacle.Length,obstacle.Width])
rectangle('Position',[obstacle.rrSafeX,obstacle.rrSafeY,obstacle.Length+2*obstacle.safeDistanceX,obstacle.Width+2*obstacle.safeDistanceY],'LineStyle','--')
yline(lanewidth/2,'b--')
yline(-lanewidth/2,'b--')
yline(-lanewidth*lanes/2,'r')
yline(lanewidth*lanes/2,'r')
% xlim([0 500])
ylabel('Y')
xlabel('X') 
title('Position')
for i=1:length(slopes)
    X=[0;50;100];
    Y=slopes(i)*X+intercepts(i);
    line(X,Y,'LineStyle','--','Color','g')
end
ylim([-6 6])
hold off
%%
figure
subplot(2,1,1)
plot(T,states(1,:))
ylabel('\dot(y)')
subplot(2,1,2)
plot(T,states(2,:))
ylabel('x__dot')
xlabel('Time (s)')
%% 
% Plot input

figure
subplot(2,1,1)
plot(T,inputs(1,:))
ylabel('\delta')
subplot(2,1,2)
plot(T,inputs(2,:))
ylabel('F_{f,l}')
xlabel('Time (s)')


figure
tiledlayout(4,1);
nexttile
plot(T,states(1,:))
legend('X')
xlabel('Time (s)')
nexttile
plot(T,states(2,:))
legend('Y')
xlabel('Time (s)')
nexttile
plot(T,states(3,:))
legend('psi')
xlabel('Time (s)')
nexttile
plot(T,states(4,:))
legend('v')
xlabel('Time (s)')
