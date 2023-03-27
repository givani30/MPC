function LTV=computeLTV(Xref,Uref,param,dim)

%computes the linearization along the given trajectory

Ad=zeros(dim.nx,dim.nx,size(Xref,1));
Bd=zeros(dim.nx,dim.nu,size(Xref,1));

for i=1:size(Xref,1)
    Ad(:,:,i) =[1                   Uref(i,2)*param.T                   0; 
                -Uref(i,2)*param.T          1           Uref(i,1)*param.T;
                0                           0                           1];
            
    Bd(:,:,i) =[-param.T 0; 0 0; 0 -param.T];
end

LTV.Ad=Ad;
LTV.Bd=Bd;