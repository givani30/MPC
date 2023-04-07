function [T,S]=predmodgen(A,B,N,n,p)

%Prediction matrices generation
%This function computes the prediction matrices to be used in the
%optimization problem

%Prediction matrix from initial state
T=zeros(n*(N+1),n);
for k=0:N
    T(k*n+1:(k+1)*n,:)=A^k;
end

%Prediction matrix from input
S=zeros(n*(N+1),p*(N));
for k=1:N
    for i=0:k-1
        S(k*n+1:(k+1)*n,i*p+1:(i+1)*p)=A^(k-1-i)*B;
    end
end


