%%
clear all
close all
clc

%% Data & Initialization

% Definition of the LTI system
LTI.A = [1 1;0 1];
LTI.B = [0;1];

% Definition of system dimension
dim.nx = 2;     % state dimension
dim.nu = 1;     % output dimension
dim.N = 2;      % prediction horizon

% State constraints
A = [1 0];
b = 2;

% Input constraints
C = [1;-1];
d = ones(2,1);

%%
% Define the set Z

% Compute state evolution x+ = P x + S u
[P,S] = predmodgen_state(LTI,dim);

% Select the first component, for each temporal step
G_x = P(1:2:end,:);
H_x = S(1:2:end,:);

Psi_x = b*ones(dim.N+1,1);

% Define input constraints
H_u = kron(eye(dim.N),C);
Psi_u = repmat(d,dim.N,1);

G = [G_x; zeros(size(H_u,1),dim.nx)];
H = [H_x; H_u];
Psi = -[Psi_x; Psi_u];

%%
% Algorithm implementation

G_i = [G H(:,1:end-1)];
H_i = H(:,end);
Psi_i = Psi;

for i = dim.N-1:-1:0

    [P_i, gamma_i] = single_input(G_i,H_i,Psi_i);
    
    G_i = P_i(:,1:end-1);
    H_i = P_i(:,end);
    Psi_i = gamma_i;
    
end

P = P_i
gamma = gamma_i






















