function [sys, U,Y,X,DX] = discreteSS(eps, u, params,Ts)
    % This function calculates the discrete LTI model of the system
    % around the equillibrium point eps and u.
    % The system is defined as:
    % eps_dot = f(eps, u) where eps_dot is the time derivative of the state
    % vector eps and f is a function of the state vector eps and the input vector u
    % The system is linearized around the equillibrium point eps and u, which
    % means that the following equation holds:
    % eps_dot = A * eps + B * u, where A is the jacobian of f evaluated around
    % the equillibrium point, B is the jacobian of f evaluated around the
    % equillibrium point and u is the input vector.
    %
    % Inputs:
    % eps: A 6x1 vector containing the equillibrium point of the system
    % u: A 5x1 vector containing the input vector u
    % params: A 5x1 vector containing the parameters of the system
    % Ts: The sampling time of the system
    %
    % Outputs:
    % sys: A state space object containing the linearized state space model of
    % the system
    %
    % Example:
    % eps = [0; 0; 0; 0; 0; 0];
    % u = [0; 0; 0; 0; 0];
    % params = [1; 1; 1; 1; 1];
    % Ts = 0.01;
    % sys = linearSS(eps, u, params, Ts);

    % Extract states
    y_dot = eps(1); % The velocity of the car in the y direction
    x_dot = eps(2); % The velocity of the car in the x direction
    psi_dot = eps(3); % The angular velocity of the car around the z axis
    psi = eps(4); % The angle between the body frame and the x axis
    Y = eps(5); % The position of the car in the y direction
    X = eps(6); % The position of the car in the x direction

    % Extract inputs
    delta = u(1); % The angle between the body frame and the front axle
    F_f = u(2); % The force applied to the front left tire
    F_r = u(3); % The force applied to the front right tire

    % Extract parameters
    g = 9.81; % gravity
    m = params(1); % mass of car
    I = params(2); % The moment of inertia of the car around the z axis
    a = params(3); % The distance from the center of mass to the front axle (x axis)
    b = params(4); % The distance from the center of mass to the rear axle (x axis)
  

    % Calculate linearization matrices

    lin_x = [0, -psi_dot, -x_dot, 0, 0, 0;
             psi_dot, 0, y_dot, 0, 0, 0;
             0, 0, 0, 0, 0, 0;
             0, 0, 1, 0, 0, 0;
             cos(psi), sin(psi), 0, x_dot * cos(psi) + y_dot * (-sin(psi)), 0, 0;
             -sin(psi), cos(psi), 0, x_dot * (-sin(psi)) - y_dot * cos(psi), 0, 0];
    Ac = lin_x;
    
    lin_u = [2*F_f*cos(delta)/m, 2*sin(delta)/m, 0;
        -2*F_f*sin(delta)/m, 2*cos(delta)/m, 2/m;
        2*F_f*a*cos(delta)/I, 2*a*sin(delta)/I 0;
        0 0 0;
        0 0 0;
        0 0 0];
    Bc = lin_u;
    %Create ss object to store results
    [Ad,Bd]=adasblocks_utilDicretizeModel(Ac,Bc,Ts);
%     C = diag([0, 0, 0, 0, 1, 1]); %output y is the position of the car in the reference frame
    C=[sin(psi) cos(psi) 0 0 0 0;
        0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1];
sys = ss(Ad, Bd, C, [],Ts);
    sys.InputName = {'delta', 'F_f', 'F_r'};
    sys.StateName = {'y_dot', 'x_dot', 'psi_dot', 'psi', 'Y', 'X'};
    sys.OutputName = {'X_dot', 'psi', 'Y','X' };
    
    U=u;
    Y=C*eps;
    X=eps;
    DX=Ad*eps+Bd*u-eps;
end
