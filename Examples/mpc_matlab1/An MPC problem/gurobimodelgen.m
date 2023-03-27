function MPC = gurobimodelgen(H_, h_, A_, b_)

    % Define a new model
    % Define the objective function
    MPC.Q = .5*sparse(H_);  % Quadratic term of the objective function
    MPC.obj = -h_';  % Linear term of the objective function
        
    % Set the constraints
    MPC.A = sparse(A_);   % Matrix of linear constraints
    MPC.rhs = b_;   % Vector of linear constraints
    MPC.sense = '<';  % Define the sense of inequality. It includes the '='.
    
    % Defining the type of variables in the model
    MPC.vtype = char(repmat({'C'},[1,size(A_,2)]))'; 
end
    