function d = delta_reg(this,x,v)
%DELTA_REG Calculate the Dynamic Safety Margin for ERGsys objects with an
%ERGmultiregion
%
%   d=sys.DELTA_REG(x,v) calculates the Dynamic Safety Margin for the
%   instance of ERGsys, sys, for a given state x and applied reference v in
%   the case that sys has a ERGmultiregion. For this to be computed, the
%   Lyapunov functions of the system must be calculated first with the
%   method calculateQuadraticRegionalLyapunov().
Acl=this.Acl;
Bcl=this.Bcl;
p=size(this.C,1);
%Calculate the steady state for v
xv=-inv(Acl)*Bcl*v;

P=this.getQuadraticRegionalLyapunov;

if isa(this.region,'ERGmultiregion')
    region=this.region.regions;
else
    %Convert to cell for compatibility
    region{1}=this.region;
end

nr=this.getRegion.whichRegion(xv,v);
l=length(nr);
for i=1:l
    ncons=size(P{nr(i)},3);
    gamma=zeros(1,ncons);
    V=zeros(1,ncons);
    for j=1:ncons
        %For every constraint in the region
        beta_x=region{nr(i)}.M(j,1:end-p)';
        beta_v=region{nr(i)}.M(j,end-p+1:end)';
        h=region{nr(i)}.b(j);
        gamma(j)=(beta_x'*xv+beta_v'*v-h)^2/(beta_x'*(P{nr(i)}(:,:,j)\beta_x));
        V(j)=(x-xv)'*P{nr(i)}(:,:,j)*(x-xv);
    end
    delta(i)=max([min(gamma-V),0]);
end
d=max(delta);
if d<0
    d=0;
end
end