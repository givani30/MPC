u=[];
t=1:1001;
for i=t
    u(i,:)=unyrefinput(i*param.T,dim);
end

plot(t,Uref,t,u)