%% 
% Define system params

m= 1000; %kg, mass of the vehicle
I=1000; %kgm^2, moment of inertia of the vehicle
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
V=50/3.6
eps_0=[0;
    V;
    0;
    0;
    0;
    0];
%% 
% MPC

%Sampling time of T_s=0.05
Ts=5e-2;
u_0=zeros(5,1);
dsys=minreal(discreteSS(eps_0,u_0,parameters,Ts));
mpcobj=mpc(dsys)
%% 
% STEPS:
%% 
% # Define prediction horizon

mpcobj.PredictionHorizon = 2/Ts %2 sec
mpcobj.ControlHorizon=4 %0.1 Sec
%% 
% # Constraints on Control (Max Throttle, Max throttle rate of change. Max steering 
% angle,)

mpcobj.ManipulatedVariables(2).RateMax=0.5*Ts; %Max throttle rate of change
mpcobj.ManipulatedVariables(3).RateMax=0.5*Ts; %Max throttle rate of change
mpcobj.ManipulatedVariables(4).RateMax=0.5*Ts; %Max throttle rate of change
mpcobj.ManipulatedVariables(5).RateMax=0.5*Ts; %Max throttle rate of change

mpcobj.ManipulatedVariables(2).RateMin=-0.5*Ts; %Min throttle rate of change
mpcobj.ManipulatedVariables(3).RateMin=-0.5*Ts; %Min throttle rate of change
mpcobj.ManipulatedVariables(4).RateMin=-0.5*Ts; %Min throttle rate of change
mpcobj.ManipulatedVariables(5).RateMin=-0.5*Ts; %Min throttle rate of change

% #Constraint on turing radius of car
mpcobj.ManipulatedVariables(1).Max=atan(2.5/5); %Max steering angle
mpcobj.ManipulatedVariables(1).Min=-atan(2.5/5); %Min steering angle

% Constraint on throttle
maxT=100
mpcobj.ManipulatedVariables(2).Max=maxT; %Max throttle [N*m]
mpcobj.ManipulatedVariables(3).Max=maxT; %Max throttle [N*m]
mpcobj.ManipulatedVariables(4).Max=maxT; %Max throttle [N*m]
mpcobj.ManipulatedVariables(5).Max=maxT; %Max throttle [N*m]

mpcobj.ManipulatedVariables(2).Min=-maxT; %Min throttle (brake) [N*m]
mpcobj.ManipulatedVariables(3).Min=-maxT; %Min throttle (brake) [N*m]
mpcobj.ManipulatedVariables(4).Min=-maxT; %Min throttle (brake) [N*m]
mpcobj.ManipulatedVariables(5).Min=-maxT; %Min throttle (brake) [N*m]
%% 
% # Scale the MV NIET NODIG?
% # Weights on output vars
mpcobj.Weights.OutputVariables=[1 1 1 0 1 0]; %Weight on x_dot,y_dot,psi and y
% # Nominal operating point
mpcobj.Model.Nominal.X=eps_0;
mpcobj.Model.Nominal.U=u_0;
mpcobj.Model.Nominal.DX=dsys.A*eps_0+dsys.B*u_0-eps_0;
mpcobj.Model.Nominal.Y=dsys.C*eps_0+dsys.D*u_0;
%%
% # Define the constraints on the output variables
%Type of constraints: E*u+F*y<=G
E1=[0 0 0 0 0];
F1=[0 0 0 0 1 0];
lanewidth=3.5;
lanes=3;
G1=lanewidth*lanes/2;
% 
E2=[0 0 0 0 0];
F2=[0 0 0 0 -1 0];
G2=lanewidth*lanes/2;
%
E3=[0 0 0 0 0];
F3=[0 0 0 0 1 0];
G3=lanewidth*lanes/2;
%
setconstraint(mpcobj, [E1;E2;E3], [F1;F2;F3], [G1;G2;G3],[1;1;0.1]);

%Simulate the system
refSpeed=[0;V;0;0;0;0];

x=eps_0;
u=u_0;

egostates=mpcstate(mpcobj)

T=0:Ts:10;

for i=1:length(T)
    [u,~,egostates]=mpcmove(mpcobj,refSpeed,[],[],egostates);
    x=dsys.A*x+dsys.B*u;
    y=dsys.C*x+dsys.D*u;
    X(i,:)=x';
    Y(i,:)=y';
    U(i,:)=u';
end