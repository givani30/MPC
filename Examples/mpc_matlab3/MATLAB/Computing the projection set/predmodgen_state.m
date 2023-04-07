function [P,S] = predmodgen_state(LTI,dim)

%Prediction matrices generation
%This function computes the prediction matrices to be used in the
%optimization problem

%Prediction matrix from initial state
P=eye(dim.nx);
for k=1:dim.N
    P=[P;LTI.A^k];
end

%Prediction matrix from input
S=zeros(dim.nx,dim.nu*dim.N);
S_prec=S;
for k=1:dim.N
   S=[S;circshift(S_prec,1,2)];
   S(end-1:end,1)=LTI.A^(k-1)*LTI.B;
   S_prec=S(end-1:end,:);
end