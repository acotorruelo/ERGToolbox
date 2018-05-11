function out = calculateQuadraticRegionalLyapunov(this,varargin)
%CALCULATEQUADRATICREGIONALLYAPUNOV Calculate the Lyapunov functions of
%every constraint, for every ERGregion in the ERGmultiregion of the ERGsys.
%
%   sys=sys.CALCULATEQUADRATICLYAPUNOV calculates the Lyapunov function for
%   every constraint in every ERGregion of the ERGmultiregion of the
%   instance of ERGsys, sys. This function returns a cell object with as
%   many ERGregions there are in the ERGmultiregion of sys.

%Retrieve system parameters
if isa(this.region,'ERGmultiregion')
    region=this.region.regions;
else
    %Convert to cell for compatibility
    region{1}=this.region;
end

if nargin>1
    solver=varargin{2};
else
    solver='sdpt3';
end

%Calculate the closed loop A
A=this.Acl;
l=length(region);
n=size(A,2);
p=size(this.C,1);
Pcell=cell(1,length(region));

Q=sdpvar(n,n,'symmetric');
fcn=-logdet(Q);
        
epsilon=1e-4;
options=sdpsettings('solver',solver,'verbose',0);
for i=1:l
    %For each region in the multiregion
    ll=size(region{i}.M,1);
    P3=zeros(n,n,ll);
    for j=1:ll
        %For each constraint in the region
        
        % LMI problem definition
        beta_x=region{i}.M(j,1:end-p)';
        cons=[
            Q*A'+A*Q<=-epsilon;
            [Q Q*beta_x;
            beta_x'*Q 1]>=0
            ];
        optimize(cons,fcn,options);
        %Store the P matrix
        P3(:,:,j)=inv(double(Q));
        if ~all(eig(double(Q))>0)
            %If not semidefinite, terminate execution
            error(['Matrix P for constraint ' num2str(i) ' is not positive semidefinite.'])
        end
    end
    Pcell{i}=P3;
end
this.Preg=Pcell;
out=this;
end