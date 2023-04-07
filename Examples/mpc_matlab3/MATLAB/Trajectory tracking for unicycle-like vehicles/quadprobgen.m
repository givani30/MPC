function quadcost=quadprobgen(predmod,weight,dim)

%% Quadratic problem generation

Q_bar=blkdiag(kron(eye(dim.N),weight.Q),weight.P);
R_bar=kron(eye(dim.N),weight.R);

H=zeros(dim.N*dim.nu,dim.N*dim.nu,dim.t);
h=zeros(dim.N*dim.nu,dim.nx,dim.t);

for k=1:dim.t
    H(:,:,k)=(R_bar+predmod.S(:,:,k)'*Q_bar*predmod.S(:,:,k));
    h(:,:,k)=predmod.S(:,:,k)'*Q_bar*predmod.T(:,:,k); 
end

quadcost.H=H;
quadcost.h=h;
