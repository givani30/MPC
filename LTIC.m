function eps_dot = LTIC(eps, u, params)
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
    F_fl = u(2); % The force applied to the front left tire
    F_fr = u(3); % The force applied to the front right tire
    F_rl = u(4); % The force applied to the rear left tire
    F_rr = u(5); % The force applied to the rear right tire

    % Extract parameters
    g = 9.81; % gravity
    m = params(1); % mass of car
    I = params(2); % The moment of inertia of the car around the z axis
    a = params(3); % The distance from the center of mass to the front axle (x axis)
    b = params(4); % The distance from the center of mass to the rear axle (x axis)
    c = params(5); % The distance from the center of mass to the left/right side of the tires (y axis)

    %Direction of tire forces
    F_x_fl=F_fl*cos(delta);
    F_y_fl=F_fl*sin(delta);
    F_x_fr=F_fr*cos(delta);
    F_y_fr=F_fr*sin(delta);
    F_x_rl=F_rl;
    F_y_rl=0;
    F_x_rr=F_rr;
    F_y_rr=0;

    %Calculate the derivatives of the states
    y_ddot=-x_dot*psi_dot+F_y_fl/m+F_y_fr/m+F_y_rl/m+F_y_rr/m;
    x_ddot=y_dot*psi_dot+F_x_fl/m+F_x_fr/m+F_x_rl/m+F_x_rr/m;
    psi_ddot=(a*(F_y_fl+F_y_fr)-b*(F_y_rl+F_y_rr)+c*(-F_x_fl+F_x_fr-F_x_rl+F_x_rr))/I;
    Y_dot=x_dot*sin(psi)+y_dot*cos(psi);
    X_dot=x_dot*cos(psi)-y_dot*sin(psi);
    eps_dot=[y_ddot; x_ddot; psi_ddot; psi_dot; Y_dot; X_dot];
end
