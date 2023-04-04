%Define the LTI system
A=[0 1 0; 0 0 1; 1 2 3];
B=[0 0 1]';

%Define the dimension of the system
dim.nx=size(A,1);
dim.nu=size(B,2);

%Define the weights
R=2;
% R=100*2:
Q=[1 0 0; 0 2 0; 0 0 0];
% Q=100*[1 0 0; 0 2 0; 0 0 0];
      

%%
%Riccati Riccati equation iterations
Pi=zeros(dim.nx);    %Initialize the matrix \Pi

% Iterate ALGEBRAIC Riccati equation till Pi stops changing
for i=1:100 
    Pi=Q+A'*Pi*A-A'*Pi*B*((B'*Pi*B+R)\B')*Pi*A;
end
K=-(B'*Pi*B+R)\B'*Pi*A;
eigLQR=eig(A+B*K);

%Check if the matrix Pi converges to the solution of DARE
[Piinf,eiginf,Kinf]=dare(A,B,Q,R);   
Kinf=-Kinf;          %see dare documentation

% figure
plot(eigLQR,'o')
viscircles([0,0],1,'Color','r','Linewidth',3);
axis square
grid minor
line([0,0], ylim, 'Color', 'k', 'LineWidth', 0.5); % Draw line for Y axis.
line(xlim, [0,0], 'Color', 'k', 'LineWidth', 0.5); % Draw line for X axis
%% Repeat for Q2=100*Q
%% Repeat for R2=R*100
