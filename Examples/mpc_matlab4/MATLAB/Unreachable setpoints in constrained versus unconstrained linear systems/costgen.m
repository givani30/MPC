function [H,h]=costgen(T,S,x0,Q,R,P,N,xstar,ustar)

xref=kron(ones(N+1,1),xstar);
uref=kron(ones(N,1),ustar);

Qbar=blkdiag(kron(eye(N),Q),P); 
Rbar=kron(eye(N),R); 

H=S'*Qbar*S+Rbar;   
h=S'*Qbar*(T*x0-xref)-Rbar*uref;

end