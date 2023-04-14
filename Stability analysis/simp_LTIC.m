function eps_dot = simp_LTIC(eps, u, params)
    % This function evaluates the continous LTI model of the system
    % around the equillibrium point eps and u.
    % The system is defined as:
    % eps_dot = f(eps, u) where eps_dot is the time derivative of the state
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
    x = eps(1); % The velocity of the car in the y direction
    y = eps(2); % The velocity of the car in the x direction
    psi = eps(3); % The angular velocity of the car around the z axis
    V= eps(4); % The position of the car in the x direction

    % Extract inputs
    delta = u(1); % The angle between the body frame and the front axle
    F=u(2);

    % Extract parameters
    g = 9.81; % gravity
    m = params(1); % mass of car
    I = params(2); % The moment of inertia of the car around the z axis
    a = params(3); % The distance from the center of mass to the front axle (x axis)
    b = params(4); % The distance from the center of mass to the rear axle (x axis)
    c = params(5); % The distance from the center of mass to the left/right side of the tires (y axis)


    %Calculate the derivatives of the states
    x_dot=cos(psi)*V;
    y_dot=sin(psi)*V;
    psi_dot=tan(delta)/(a+b)*V;
    V_dot=0.5*F;
    eps_dot=[y_dot;psi_dot;V_dot];
end
