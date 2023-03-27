function y = outputfun(eps, u,params)
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


    y=[x_dot;
        psi;
        X;
        Y];
end
