%Define the LTI system
LTI.A=[4/3 -2/3; 1 0];
LTI.B=[1 0]';
LTI.C=[-2/3 1];
LTI.x0=[1 1]';

%Define the dimension of the system and horizon
dim.N=5;
%dim.N=15;
dim.nx=size(LTI.A,1);
dim.ny=size(LTI.C,1);
dim.nu=size(LTI.B,2);

%Define the weights for the LQR problem
param.R=1e-3;
param.Q=LTI.C'*LTI.C+param.R*eye(dim.nx);
param.P=param.Q;
% param.P=Piinf;        

%%
[T,S]=predmodgen(LTI,dim);                 
[H,h,const]=costgen(T,S,param,dim,LTI);     

%%
%Solve the quadratic optimization problem for un
cvx_begin 
    variable uN(dim.nu*dim.N)
    minimize(0.5*uN'*H*uN+h'*uN)
cvx_end

%%
%Solve the finite horizon LQR problem using Riccati backward equation
Pi=zeros(dim.nx,dim.nx,dim.N+1);    %Allocate the tridimensional matrix \Pi
K=zeros(dim.nu,dim.nx,dim.N);       %Allocate the tridimensional matrix \K

Pi(:,:,dim.N+1)=param.P;            %Initialize \Pi for the final instant

%Execute backward Riccati iteration
for k=dim.N:-1:1
    K(:,:,k) =-(LTI.B'*Pi(:,:,k+1)*LTI.B+param.R)\LTI.B'*Pi(:,:,k+1)*LTI.A;
    Pi(:,:,k)=param.Q+LTI.A'*Pi(:,:,k+1)*(LTI.A+LTI.B*K(:,:,k));
end
    
% ustar0=K(:,:,1)*LTI.x0:            %Compute the best output at k=0,by Riccati iteration
% uN(1):                             %Compute the best output at k=0,by quadratic problem solution
% eigLQR=eig(LTI.A+LTI.B*K(:,:,1));  %Compute the closed loop eigenvalues 
%%
rank(ctrb(LTI.A,LTI.B));             %Check for controllability of the system

%Compute the infinite horizon LQR solution
[Piinf,eiginf,Kinf] = dare(LTI.A,LTI.B,param.Q,param.R); 

%% Repeat the first part taking param.P=Piinf

