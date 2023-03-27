function u=discreteref(t,dim,param,Uref)

persistent k;
if isempty(k)
    k=0;
end
   
persistent uk;
if isempty(uk)
    uk=[0;0];
end

if k*param.T<t
    k=k+1;
    uk=Uref(k,:)'; 
end

u=uk;
