load('matlab.mat');
plot(x(:,1),x(:,2),Xref(:,1),Xref(:,2),'-.');
grid minor; 

xlabel('x');
ylabel('y');
legend('q','q_r')
