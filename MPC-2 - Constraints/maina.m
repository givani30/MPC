%% 
close all
clear all 
clc
% Define system parameters
m = 1000;   % kg, mass of the vehicle
I = 1500;   % kgm^2, moment of inertia of the vehicle
a = 1.5;    % m, distance from the center of mass to the front axle
b = 1.5;    % m, distance from the center of mass to the rear axle
c = 1;      % m, distance from the center of mass to the left/right side of the tires (y axis)
parameters = [m; I; a; b; c];

% Define equilibrium state with speed of 50 km/h
V = 20;
eps_0 = [0; 0; 0; 0.9 * V];

% Define lane width and number of lanes
lane_width = 3.5; 
num_lanes = 3;

% Define sampling time
Ts = 5e-2;

% Define initial inputs
u_0 = zeros(2, 1);

% Define discrete state space model and MPC object
[dsys, U, Y, X, DX] = discreteSS(eps_0, u_0, parameters, Ts);
mpcobj = mpc(dsys);

% Set prediction horizon and control horizon
mpcobj.PredictionHorizon = 60;    % 2 sec
mpcobj.ControlHorizon = 2;        % 0.1 sec

% Set constraints on control inputs
mpcobj.ManipulatedVariables(2).RateMin = -0.2 * Ts;
mpcobj.ManipulatedVariables(2).RateMax = 0.2 * Ts;
mpcobj.ManipulatedVariables(1).RateMin = -pi / 30 * Ts;
mpcobj.ManipulatedVariables(1).RateMax = pi / 30 * Ts;
mpcobj.ManipulatedVariables(1).ScaleFactor = 0.2;
mpcobj.ManipulatedVariables(2).ScaleFactor = 2;

% Set weights on output variables and manipulated variables
mpcobj.Weights.OutputVariables = [0 30 0.1 1]; % weight on x_dot, y_dot, psi, and y
mpcobj.Weights.ManipulatedVariables = [1 1];

% Set nominal operating point
mpcobj.Model.Nominal.X = X;
mpcobj.Model.Nominal.U = U;
mpcobj.Model.Nominal.DX = DX;
mpcobj.Model.Nominal.Y = Y;

% Define constraints on the output variables
obstacle = createObstacle();
[E, F, G] = baseConstraints(lane_width, num_lanes);
setconstraint(mpcobj, E, F, G, [1; 1; 0.1]);

% Define variables to store simulation data
T = 0:Ts:10;
x = eps_0;
u = u_0;
y = dsys.C * eps_0;
egostates = mpcstate(mpcobj);
states = zeros(length(x), length(T));
inputs = zeros(length(u), length(T));
detected = zeros(1, length(T));
slopes = zeros(1, length(T));
intercepts = zeros(1, length(T));
costs = zeros(1, length(T));

% Simulate the system

for i = 1:length(T)
    % Update plant states
    [newsys, U, Y, X, DX] = discreteSS(x, u, parameters, Ts);
    
    % Update constraints
    newNominal=struct('X',X,'U',U,'DX',DX,'Y',Y);
    measurements=newsys.C*x+newsys.D*u;
    opt=mpcmoveopt;
    %ADD updated constraints here
    detect=ObstacleDetect(x,obstacle);
    detected(i)=detect;
%     detect=false;
    [E,F,G,slopes(i),intercepts(i)]=updateConstraints(x,obstacle,detect,lane_width,num_lanes);
    opt.CustomConstraint=struct('E',E,'F',F,'G',G);
%Update ref speed
    refSpeed=[0;0; 0; V];
    %Get the optimal control action
    [u,info]=mpcmoveAdaptive(mpcobj, egostates, newsys, newNominal, measurements, refSpeed, [],opt);
    costs(i)=info.Cost;
    %Time update of the system
    x=egostates.Plant;
    %Save the results
    states(:,i)=x;
    inputs(:,i)=u;
end

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
yline(lane_width/2,'b--')
yline(-lane_width/2,'b--')
yline(-lane_width*num_lanes/2,'r')
yline(lane_width*num_lanes/2,'r')
% xlim([0 500])
ylabel('Y')
xlabel('X') 
title('Position')
for i=1:length(slopes)
    X=[0;50;200];
    Y=slopes(i)*X+intercepts(i);
    line(X,Y,'LineStyle','--','Color','g')
end
ylim([-6 6])
xlim([0 200])
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
%% 
figure
semilogy(states(1,:),costs)
% ylim([0 100])
xlim([0 100])
