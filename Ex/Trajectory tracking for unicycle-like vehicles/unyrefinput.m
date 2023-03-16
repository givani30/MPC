function ur=unyrefinput(t,dim)

xrdot=-sin(t);    
yrdot= 3*cos(3*t);
xrddot=-cos(t);
yrddot=-9*sin(3*t);
       
ur=zeros(dim.nu,1);
ur(1)=(xrdot^2+yrdot^2)^0.5;
ur(2)=(xrdot*yrddot-yrdot*xrddot)/(xrdot^2+yrdot^2);

end
