function out = removeConstraint(this,n)
%REMOVECONSTRAINT Removes specified constraint(s) from an ERGsys object.
%
%   sys=sys.REMOVECONSTRAINT(n) removes the constraint(s) specified by the
%   vector of indices n.
nc=size(beta_x,2);
if all(n<1) || any(isnan(n)) || any(isempty(n)) || any(isinf(n)) || any(floor(n)~=n)
    %If n is not a positive scalar
    error('n must be a strictly positive integer.')
elseif n>nc
    %If n is greater th
    error(['The system has ' num2str(nc) ' constraints.'])
elseif length(n)~=length(unique(n))
    %If n has any repeated components
    error('There are duplicates in n')
end
this.beta_x(:,n)=[];
this.beta_v(:,n)=[];
this.h(n)=[];
out=this;
end

