%% nMPC simulation.
%It makes use of the solver nmpc. The functions that define the nMPC
%problem are at the end of the script. 

clear all;
close all;
clc


%% Set parameters

Tf            = 30;                                       %simulation length  
N             = 13;                                       %control horizon
T             = 1;                                        %sample time           
x0            = [0.0, 0.5];                               %Initial conditions

%% Other solver options

t0            = 0; 
u0            = 0.2*ones(1,N);
tol_opt       = 1e-8;
opt_option    = 0;
iprint        = 5;
type          = 'difference equation';
atol_ode_real = 1e-12;
rtol_ode_real = 1e-12;
atol_ode_sim  = 1e-4;
rtol_ode_sim  = 1e-4;

%% nMPC simulation

[t,x,u]=nmpc(@runningcosts, @terminalcosts, @constraints, ...
     @terminalconstraints, @linearconstraints, @system, ...
     Tf, N, T, t0, x0, u0, ...
     tol_opt, opt_option, ...
     type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
     iprint, @printHeader, @printClosedloopData, @plotTrajectories);

%% Plots of the results 

    figure(1);
        title('x_1/x_2 closed loop trajectory');
        xlabel('x_1');
        ylabel('x_2');
        grid on;
        hold on;
        plot(sin(0:pi/20:pi), cos(0:pi/20:pi)/2, 'Color',[0.8 0.8 0.8],'LineWidth',2);
        plot(0, -0.5,'ok','MarkerSize',8);
        plot(x(:,1),x(:,2),'or','MarkerFaceColor','r');
        axis([-0.5 1.5 -1 1]);
        axis square;

    figure(2);
        title(['x_1 and x_2 closed loop trajectory']);
        xlabel('n');
        ylabel('x_1(n), x_2(n)');
        grid on;
        hold on;
        plot(t,x(:,1),'-ok');
        plot(t,x(:,2),'-ok');
        axis([0 Tf -0.5 1]);
        axis square;

%% Definition of the NMPC problem (dynamics, constraints, costs)

function cost = runningcosts(t, x, u)
    cost = norm(x-[0, -0.5],2)^2+norm(u,2)^2;
end

function cost = terminalcosts(t, x)
    cost = 0.0;
end

function [c,ceq] = constraints(t, x, u)
    c   = [];
    ceq = [];
end

function [c,ceq] = terminalconstraints(t, x)
    c   = [];
    ceq = [];
%     ceq = [x(2)+0.5];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = 0;
    ub  = [];
end

function y = system(t, x, u, T)
    xx = [x(1), 2*x(2)];
    n = norm(xx,2);
    if (n == 0.0)
        dx = [0.0 0.0];
    else
        phi = real(acos(xx(2)/n));
        if (xx(1) < 0)
        phi = 2*pi-phi;
        end
        y(1) = n*sin(phi+u(1));
        y(2) = n*cos(phi+u(1))/2;
    end
end

%% Definition of output format

function printHeader()
end

function printClosedloopData(mpciter, u, x, t_Elapsed)
end

function plotTrajectories(dynamic, system, T, t0, x0, u,atol_ode, rtol_ode, type)
end