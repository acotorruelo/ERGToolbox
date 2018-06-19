function this = calculateQuadraticLyapunov_poly(this,varargin)
%CALCULATEQUADRATICREGIONALLYAPUNOV Calculate the Lyapunov functions of
%every constraint, for every Polyhedron instance in the union
%of polyhedra assigned to the ERGContoller.
%
%   erg.CALCULATEQUADRATICREGIONALLYAPUNOV calculates the Lyapunov function for
%   every constraint in every Polyhedron of the union of polyhedra of the
%   instance of ERGController, erg.

if nargin>1
    solver=varargin{2};
else
    solver='sdpt3';
end

%Calculate the closed loop A
l=length(this.poly);
n=size(this.Acl,2);
p=size(this.C,1);
Pcell=cell(1,l);

Q=sdpvar(n,n,'symmetric');
fcn=-logdet(Q);

epsilon=1e-4;
options=sdpsettings('solver',solver,'verbose',0);
for i=1:l
    %For each region in the multiregion
    ll=size(this.poly(i).H,1);
    P3=zeros(n,n,ll);
    for j=1:ll
        %For each constraint in the region
        
        % LMI problem definition
        mbeta_x=this.poly(i).H(j,1:end-p-1)';
        cons=[
            Q*this.Acl'+this.Acl*Q<=-epsilon;
            [Q Q*mbeta_x;
            mbeta_x'*Q 1]>=0
            ];
        optimize(cons,fcn,options);
        %Store the P matrix
        P3(:,:,j)=inv(double(Q));
        if ~all(eig(double(Q))>0)
            %If not semidefinite, terminate execution
            error(['Matrix P for Polyhedron ' num2str(i) ', constraint ' num2str(j) ' is not positive semidefinite.'])
        end
    end
    Pcell{i}=P3;
end
this.Preg=Pcell;
end
