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
V=10;
eps_0=[0;
    V;
    0;
    0;
    0;
    0];
lanewidth=3.5;
lanes=3;
%% 
% MPC

%Sampling time of T_s=0.05
Ts=5e-2;
u_0=zeros(5,1);
[dsys,U,Y,X,DX]=(discreteSS(eps_0,u_0,parameters,Ts));
mpcobj=mpc(dsys);
%% 
% STEPS:
%% 
% # Define prediction horizon

mpcobj.PredictionHorizon = 45; %2 sec
mpcobj.ControlHorizon=10; %0.1 Sec
%% 
% # Constraints on Control (Max Throttle, Max throttle rate of change. Max steering 
% angle,)
% Throttlerate=10;
% mpcobj.ManipulatedVariables(2).RateMax=Throttlerate; %Max throttle rate of change
% mpcobj.ManipulatedVariables(3).RateMax=Throttlerate; %Max throttle rate of change
% 
% mpcobj.ManipulatedVariables(2).RateMax=5*Ts; %Max throttle rate of change
% mpcobj.ManipulatedVariables(3).RateMax=5*Ts; %Max throttle rate of change
% mpcobj.ManipulatedVariables(4).RateMax=5*Ts; %Max throttle rate of change
% mpcobj.ManipulatedVariables(5).RateMax=5*Ts; %Max throttle rate of change
% 
% mpcobj.ManipulatedVariables(2).RateMin=-5*Ts; %Min throttle rate of change
% mpcobj.ManipulatedVariables(3).RateMin=-5*Ts; %Min throttle rate of change
% mpcobj.ManipulatedVariables(4).RateMin=-5*Ts; %Min throttle rate of change
% mpcobj.ManipulatedVariables(5).RateMin=-5*Ts; %Min throttle rate of change

% #Constraint on turing radius of car
max_angle=1.5; %Max steering angle
% mpcobj.MV(1).RateMax=pi/30;
% mpcobj.MV(1).RateMin=-pi/30;
mpcobj.ManipulatedVariables(1).Max=max_angle; %Max steering angle
mpcobj.ManipulatedVariables(1).Min=-max_angle; %Min steering angle

% Constraint on throttle
maxT=5000;
mpcobj.ManipulatedVariables(2).Max=maxT; %Max throttle [N*m]
mpcobj.ManipulatedVariables(3).Max=maxT; %Max throttle [N*m]
mpcobj.ManipulatedVariables(4).Max=maxT; %Max throttle [N*m]
mpcobj.ManipulatedVariables(5).Max=maxT; %Max throttle [N*m]

mpcobj.ManipulatedVariables(2).Min=-maxT; %Min throttle (brake) [N*m]
mpcobj.ManipulatedVariables(3).Min=-maxT; %Min throttle (brake) [N*m]
mpcobj.ManipulatedVariables(4).Min=-maxT; %Min throttle (brake) [N*m]
mpcobj.ManipulatedVariables(5).Min=-maxT; %Min throttle (brake) [N*m]

%TODO scale MV
mpcobj.ManipulatedVariables(2).ScaleFactor=2; %Scale throttle
mpcobj.ManipulatedVariables(3).ScaleFactor=2; %Scale throttle
mpcobj.ManipulatedVariables(4).ScaleFactor=2; %Scale throttle
mpcobj.ManipulatedVariables(5).ScaleFactor=2; %Scale throttle
mpcobj.OV(3).ScaleFactor=1e10;
% mpcobj.OV(5).ScaleFactor=1e6;
% mpcobj.OV(3).Min=-lanewidth*lanes/2;
% mpcobj.OV(3).Max=lanewidth*lanes/2;

mpcobj.ManipulatedVariables(1).ScaleFactor=0.2; %Scale steering angle
%% ;

% # Weights on output vars
mpcobj.Weights.ManipulatedVariablesRate=[0.1 1 1 1 1];
mpcobj.Weights.OutputVariables=[1e3 10 1e7 0]; %Weight on x_dot,y_dot,psi and y
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
[E,F,G]=baseConstraints(lanewidth,lanes);
setconstraint(mpcobj, E,F,G,[0.1;0.1;0.01]);

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

T=0:Ts:30;

%Vars to store simulation data
states=zeros(length(x),length(T));
inputs=zeros(length(u),length(T));
detected=zeros(1,length(T));
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
    [E,F,G]=updateConstraints(x,obstacle,detect,lanewidth,lanes);
    opt.CustomConstraint=struct('E',E,'F',F,'G',G);
%Update ref speed
    refspeed=[V 0 0 0];
    %Get the optimal control action
    [u]=mpcmoveAdaptive(mpcobj, egostates, newsys, newNominal, measurements, refSpeed, [],opt);
%     [u]=mpcmove(mpcobj,egostates,measurements,refspeed)
    %Time update of the system
    x=egostates.Plant;
%     x=newsys.A*x+newsys.B*u;
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
plot(states(6,:),states(5,:))
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
% xlim([0 100])
ylabel('Y')
xlabel('X') 
title('Position')
hold off
%%
figure
subplot(2,1,1)
plot(T,states(1,:))
ylabel('y__dot')
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
subplot(1,1,1)
plot(T,states)
legend(['y_dot','x_dot','psi_dot','psi','X','Y'])
xlabel('Time (s)')