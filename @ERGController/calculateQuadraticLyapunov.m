function this = calculateQuadraticLyapunov(this,varargin)
%CALCULATEQUADRATICLYAPUNOV Calculate Lyapunov functions for every
%constraint in the ERGController object.
%
%   erg.CALCULATEQUADRATICLYAPUNOV calculates the Lyapunov function for
%   every constraint in the instance of ERGController, sys. This function returns
%   a cell object with as many entries as constraints there are in sys. If
%   sys has no constraints, it calculates the largest ellipsoid (x'*P*x<=1)
%   such that Acl'*P + P*Acl < 0

if nargin>1
    solver=varargin{2};
else
    solver='sdpt3';
end

%Calculate the closed loop A
l=length(this.h);
n=size(this.Acl,2);
%Create the output variable
if ~l
    Pcell=cell(1);
else
    Pcell=cell(1,l);
end

% LMI problem definition

Q=sdpvar(n,n,'symmetric');
fcn=-logdet(Q);
epsilon=1e-4;
options=sdpsettings('solver',solver,'verbose',0);
if ~l
    %If there are no constraints, calculate the solution to the Lyapunov
    %equation
    cons=[
        Q*this.Acl'+this.Acl*Q<=-epsilon
        Q>=epsilon;
        ];
    
    optimize(cons,fcn,options);
    Pcell{1}=double(Q);
else
    %If there are constraints, calculate the Lyapunov function for every
    %constraint
    for i=1:l
        cons=[
            Q*this.Acl'+this.Acl*Q<=-epsilon
            [Q Q*this.beta_x(:,i);
            this.beta_x(:,i)'*Q 1]>=0
            ];
        optimize(cons,fcn,options);
        %Store the P matrix
        Pcell{i}=inv(double(Q));
        if ~all(eig(Pcell{i})>0)
            %If not semidefinite, terminate execution
            error(['Matrix P for constraint ' num2str(i) ' is not positive semidefinite.'])
        end
    end
end
this.P=Pcell;
end