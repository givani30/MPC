function dx=dynamics(t,x,u,param)
    
    dx=zeros(2,1);
    dx(1)=x(2);
    dx(2)=param.g/param.l*sin(x(1))+1/(param.m*param.l^2)*feval(u,t,x);
    
end