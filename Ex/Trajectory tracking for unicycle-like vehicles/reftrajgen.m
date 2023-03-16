function Xref=reftrajgen(param,dim)

%Generate reference input (theta computed s.t. the trajectory is compatible
%with dynamics)

Xref=zeros(length(param.tspan),7);

for i=0:length(param.tspan)-1
    t=param.T*i;
    Xref(i+1,:)=[
               cos(t);                                 %xr
               sin(3*t);                               %yr
               -sin(t);                                %xrd
               3*cos(3*t);                             %yrd
               -cos(t);                                %xrdd
               -9*sin(3*t);                            %yrdd
               atan2(3*cos(3*t),-sin(t))               %\theta     
               ]';
end
