function [H,h]=costgen(predmod,weight,dim)

Qbar=blkdiag(kron(eye(dim.N),weight.Q),zeros(dim.nx)); 
H=predmod.S'*Qbar*predmod.S+kron(eye(dim.N),weight.R);   
h=predmod.S'*Qbar*predmod.T;