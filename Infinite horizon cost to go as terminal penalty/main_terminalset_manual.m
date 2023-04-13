%% "Infinite horizon cost to go as terminal penalty"
clear all
close all
clc

%% Model definition
m= 1000; %kg, mass of the vehicle
I=1500; %kgm^2, moment of inertia of the vehicle
a=1.5; %m, distance from the center of mass to the front axle
b=1.5; %m, distance from the center of mass to the rear axle
c=1; % The distance from the center of mass to the left/right side of the tires (y axis)
parameters=[m;I;a;b;c];
%% 
V=5;
eps_0=[0;
    0;
    0.01;
    V];
u0=[0.01;0];
Ts=1e-2;
% Define model, cost function, and bounds.
sys=discreteSS(eps_0,u0,parameters,Ts);
A = sys.A;
B = sys.B;
N = 3;

% alpha = 1e-5;
Q = diag([0.1 30 0.1 1]);
R = 0.01*eye(2);

% Bounds.
xlb = [-1000; -3.5*1.5; -100;-20];
xub = [1000; ;100;20];
ulb = [-inf; -pi];
uub = [inf; pi];

% Find LQR.

[K, P] = dlqr(A, B, Q, R);
K = -K; % Sign convention.
%%

x1_min = -1000; x1_max = 1000;
x2_min = -3.5*1.5; x2_max = 3.5*1.5;
x3_min = -pi; x3_max = pi;
x4_min = -20; x4_max = 20;
Xf = [x1_min x1_max; x2_min x2_max; x3_min x3_max; x4_min x4_max];
A_Xf = [A; -A];
b_Xf = [x1_max; x2_max; x3_max; x4_max; -x1_min; -x2_min; -x3_min; -x4_min];

% Check feasibility for each corner point of Xf
x1_vals = [x1_min x1_max];
x2_vals = [x2_min x2_max];
x3_vals = [x3_min x3_max];
x4_vals = [x4_min x4_max];
for i=1:2
    for j=1:2
        for k=1:2
            for l=1:2
                x = [x1_vals(i); x2_vals(j); x3_vals(k); x4_vals(l)];
                if all(A_Xf*x <= b_Xf)
                    disp(['Corner point [' num2str(x(1)) ',' num2str(x(2)) ',' num2str(x(3)) ',' num2str(x(4)) '] is feasible.']);
                    % Calculate the new LQR gain K for this point
                    [K_new,S_new,e_new] = dlqr(A,B,Q,R);
                    % Check whether the updated state is still in the feasible set
                    x_new = A*x + B*K_new*x;
                    if all(A_Xf*x_new <= b_Xf)
                        disp(['Updated state [' num2str(x_new(1)) ',' num2str(x_new(2)) ',' num2str(x_new(3)) ',' num2str(x_new(4)) '] is feasible.']);
                    else
                        disp(['Updated state [' num2str(x_new(1)) ',' num2str(x_new(2)) ',' num2str(x_new(3)) ',' num2str(x_new(4)) '] is not feasible.']);
                    end
                else
                    disp(['Corner point [' num2str(x(1)) ',' num2str(x(2)) ',' num2str(x(3)) ',' num2str(x(4)) '] is not feasible.']);
                end
            end
        end
    end
end