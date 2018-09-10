function this = computeIndividualLyap(this,beta_x,varargin)
%COMPUTEINDIVIDUALLYAP Calculate Lyapunov functions for every
%constraint in the ERGController object.
%
%   erg.COMPUTEINDIVIDUALLYAP computes the Lyapunov function for
%   every constraint in the instance of ERGController, sys. This function returns
%   a cell object with as many entries as constraints there are in sys. If
%   sys has no constraints, it calculates the largest ellipsoid (x'*P*x<=1)
%   such that Acl'*P + P*Acl < 0

if nargin>2
    solver=varargin{2};
else
    solver=[];
end

%Calculate the closed loop A

n=size(this.Acl,2);

% LMI problem definition

Q=sdpvar(n,n,'symmetric');
fcn=-logdet(Q);
epsilon=1e-4;
if ~isempty(solver)
    options=sdpsettings('solver',solver,'verbose',0);
else
    options=sdpsettings('verbose',0);
end
%If there are constraints, calculate the Lyapunov function for every
%constraint
cons=[
    Q*this.Acl'+this.Acl*Q<=-epsilon
    [Q Q*beta_x;
    beta_x'*Q 1]>=0
    ];
optimize(cons,fcn,options);
%Store the P matrix
P=inv(double(Q));
if ~all(eig(P)>0)
    %If not semidefinite, terminate execution
    error('Matrix P for constraint is not positive semidefinite.')
end

this.P{length(this.P)+1}=P;
end