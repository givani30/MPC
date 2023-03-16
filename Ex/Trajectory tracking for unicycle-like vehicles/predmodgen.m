function predmod=predmodgen(LTV,dim)

%% Prediction matrices generation
%This function computes the prediction matrices to be used in the quadratic
%problem generation, for a linear system varying both at each time step and
% along the prediction horizon

%%
Ad=LTV.Ad;
Bd=LTV.Bd;

%% Prediction matrices generation

%prediction matrix from input (model varying along prediction horizon)
S_bar=zeros(dim.nx*dim.N+1,dim.nu*dim.N,dim.t);
for k=1:dim.t
    mem=zeros(dim.nx,dim.nu*dim.N);
    for i=1:dim.N
        S_bar(i*dim.nx+1:(i+1)*dim.nx,:,k)=Ad(:,:,k+i-1)*mem;
        S_bar(i*dim.nx+1:(i+1)*dim.nx,(i-1)*dim.nu+1:i*dim.nu,k)=Bd(:,:,k+i-1);
        mem=S_bar(i*dim.nx+1:(i+1)*dim.nx,:,k);
    end
end

%prediction matrix from initial state (model varying along prediction horizon)
T_bar=zeros(dim.nx*dim.N+1,dim.nx,dim.t);
for k=1:dim.t
    mem=eye(dim.nx);
    T_bar(1:dim.nx,:,k)=mem;
    for i=1:dim.N
        T_bar(i*dim.nx+1:(i+1)*dim.nx,:,k)=Ad(:,:,k+i-1)*mem;
        mem=Ad(:,:,k+i-1)*mem;
    end
end

predmod.S=S_bar;
predmod.T=T_bar;


end
