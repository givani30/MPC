function [P,S]=predmodgen_output(LTI,dim)

%Prediction matrices generation
%This function computes the prediction matrices to be used in the
%optimization problem

%Prediction matrix from initial state
P=LTI.C;
for k=1:dim.N
    P=[P;LTI.C*LTI.A^k];
end

%Prediction matrix from input
S=zeros(dim.ny,dim.nu*dim.N);
for k=1:dim.N
   S=[S; circshift(S(k,:),1,2)];
   S(k+1,1)=LTI.C*LTI.A^(k-1)*LTI.B;
end

