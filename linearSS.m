function [A,B,C,D] = linearSS(eps, u, params)
    % This function calculates the continuous LTI model of the system
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
    X = eps(1); % The velocity of the car in the y direction
    Y = eps(2); % The velocity of the car in the x direction
    psi = eps(3); % The angular velocity of the car around the z axis
    V= eps(4); % The angle between the body frame and the x axis
  
    % Extract inputs
    delta = u(1); % The angle between the body frame and the front axle
    F = u(2); % The force applied to the front left tire
   
    % Extract parameters
    g = 9.81; % gravity
    m = params(1); % mass of car
    I = params(2); % The moment of inertia of the car around the z axis
    a = params(3); % The distance from the center of mass to the front axle (x axis)
    b = params(4); % The distance from the center of mass to the rear axle (x axis)
    c = params(5); % The distance from the center of mass to the left/right side of the tires (y axis)

    % Calculate linearization matrices

    lin_x = [ 0, 0, -V*sin(psi), cos(psi);
      0, 0,  V*cos(psi), sin(psi);
      0, 0, 0,             tan(delta)/(a+b);
      0, 0, 0,             0];
    A = lin_x;
    
    lin_u =  [0  , 0;
     0  , 0;
     (V*(tan(delta)^2 + 1))/(a+b), 0;
     0, 0.5];
    B = lin_u;
    %Create ss object to store results
    C=eye(4);
    D = zeros(4,2);
%     sys = ss(Ac,Bc,C,D);
%     sys.InputName = {'delta', 'F'};
%     sys.StateName = {'X', 'Y', 'psi', 'V'};
%     sys.OutputName = sys.StateName;

end
