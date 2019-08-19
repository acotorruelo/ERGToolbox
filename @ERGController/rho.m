%Navigation field for convex constraints
function out = rho(this,r,v)
%RHO Returns the Navigation Field for a system with a convex set of
%constraints
%   erg.RHO(r,v) calculates the Navigation Field for an
%   instance of ERGController erg, with reference r and applied reference
%   v.
if (this.eta<=0)||(this.zeta<=0)||(this.delta<=0)
    error('The value of all parameters must be greater than zero.')
elseif this.delta>this.zeta
    error('Parameter zeta must be greater than delta.')
end
att=(r-v)/max([norm(r-v) this.eta]);
rep=zeros(size(v));
nc=size(this.beta_x,2);
xv=this.CL*v;
for i=1:nc
    c=-this.beta_x(:,i)'*xv-this.beta_v(:,i)'*v+this.h(i);
    cv=(-this.beta_x(:,i)'*this.CL-this.beta_v(:,i)')';
    if norm(cv)~=0
        rep=rep+max((this.zeta-c)/(this.zeta-this.delta),0)*cv/norm(cv);
    end
end
out=att+rep;