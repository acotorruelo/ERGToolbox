function d = delta_poly(this,x,v)
%DELTA_POLY Calculate the Dynamic Safety Margin for ERGController
%objects constrained within a union of polyhedra.
%
%   d=erg.DELTA_POLY(x,v) calculates the Dynamic Safety Margin for the
%   instance of ERGController, erg, for a given state x and applied reference v in
%   the case that erg has a union of polyhedra. For this to be computed, the
%   Lyapunov functions of the system must be calculated first with the
%   method calculateQuadraticRegionalLyapunov().

p=size(this.C,1);
%Calculate the steady state for v
xv=-inv(this.Acl)*this.Bcl*v;

nr=this.whichPoly(xv,v);
l=length(nr);
for i=1:l
    ncons=size(this.Preg{nr(i)},3);
    gamma=zeros(1,ncons);
    V=zeros(1,ncons);
    for j=1:ncons
        %For every constraint in the region
        mbeta_x=this.poly(nr(i)).H(j,1:end-p-1)';
        mbeta_v=this.poly(nr(i)).H(j,end-p:end-1)';
        mh=this.poly(nr(i)).H(j,end);
        gamma(j)=(mbeta_x'*xv+mbeta_v'*v-mh)^2/(mbeta_x'*(this.Preg{nr(i)}(:,:,j)\mbeta_x));
        V(j)=(x-xv)'*this.Preg{nr(i)}(:,:,j)*(x-xv);
    end
    delta(i)=max([min(gamma-V),0]);
end
d=max(delta);
if d<0
    d=0;
end
end
