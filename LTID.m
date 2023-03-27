function xk1 = LTID(x,u,params,Ts)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
M = 10;
delta = Ts/M;
xk1 = x;
for ct=1:M
    xk1 = xk1 + delta*LTIC(xk1,u,params);
end
end