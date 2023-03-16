function [H,h,const]=costgen(T,S,param,dim,LTI)

Qbar=blkdiag(kron(eye(dim.N),param.Q),param.P); 

H=S'*Qbar*S+kron(eye(dim.N),param.R);   
h=S'*Qbar*T*LTI.x0;
const=LTI.x0'*T'*Qbar*T*LTI.x0;

end