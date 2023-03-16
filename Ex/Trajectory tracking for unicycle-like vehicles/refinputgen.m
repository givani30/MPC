function Uref=refinputgen(Xref,param,dim)


%Generate reference input (by explicit expression obtained by dynamics)
Uref=zeros(size(Xref,1),2);

for i=1:size(Xref,1)
    t=(i-1)*param.T;
    xrdot=-sin(t);
    yrdot=3*cos(3*t);
    xrddot=-cos(t);  
    yrddot=-9*sin(3*t); 

    ur=zeros(dim.nu,1);
    ur(1)=(xrdot^2+yrdot^2)^0.5;
    ur(2)=(xrdot*yrddot-yrdot*xrddot)/(xrdot^2+yrdot^2);
    
    Uref(i,:)=ur; 
end