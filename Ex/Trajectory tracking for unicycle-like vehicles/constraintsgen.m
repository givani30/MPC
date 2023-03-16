function constraints=constraintsgen(dim)

constraints.LB=kron(-100*ones(dim.N,1),[1;0]);
constraints.UB=kron(100*ones(dim.N,1),[1;0]);