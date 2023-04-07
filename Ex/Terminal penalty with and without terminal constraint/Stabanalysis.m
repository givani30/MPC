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