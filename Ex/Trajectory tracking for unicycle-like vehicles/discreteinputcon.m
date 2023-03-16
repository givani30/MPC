function u=discreteinputcon(x,t,dim,quadcost,param,Uref,Xref,constraints)

%% Definition of persisten variables
%Since the system is continuous time but the input changes at discrete
%instants, you have to keep the save of the input in between sampling
%times

persistent k;
if isempty(k)
    k=1;
end
   
persistent uk;
if isempty(uk)
    uk=[0;0];
end

    
%% Only update the input at discrete times

if k*param.T<=t   
    k=k+1;
    %compute tracking error
    e=zeros(3,1);
    e(1:2)=[cos(x(3)) sin(x(3)); -sin(x(3)) cos(x(3))]*(Xref(k-1,1:2)'-x(1:2));
    e(3)=Xref(k-1,7)-wrapToPi(x(3));     %atan2(x,y) is in [-pi pi]
    
    
    LB=constraints.LB-kron(Uref(k:k+dim.N-1,2),[1;0]);
    UB=constraints.UB-kron(Uref(k:k+dim.N-1,2),[1;0]);
    uB=quadprog(quadcost.H(:,:,k-1),quadcost.h(:,:,k-1)*e,[],[],[],[],LB,UB);
    uk=uB(1:dim.nu);
    k
end

u=uk+Uref(k,:)';  %uB(k)+uF(k)
