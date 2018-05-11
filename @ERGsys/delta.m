function d = delta(this,x,v)
%DELTA Calculate the Dynamic Safety Margin.
%
%   d=sys.DELTA(x,v) calculates the Dynamic Safety Margin for the instance
%   of ERGsys, sys, for a given state x and applied reference v. For this
%   to be computed, the Lyapunov functions of the system must be calculated
%   first with the method calculateQuadraticLyapunov().

%Get the system parameters
P=this.P;
h=this.h;
beta_x=this.beta_x;
beta_v=this.beta_v;
Acl=this.Acl;
Bcl=this.Bcl;

%Calculate the steady state for v
xv=-inv(Acl)*Bcl*v;


nc=size(beta_x,2);
gammav=zeros(1,nc);
Vv=gammav;

%Calculate gamma and V for every constraint
for i=1:nc
    gammav(i)=(beta_x(:,i)'*xv+beta_v(:,i)'*v-h(i))^2/(beta_x(:,i)'*(P{i}\beta_x(:,i)));
    Vv(i)=(x-xv)'*P{i}*(x-xv);
end

%Return the minimum value of the Dynamic Safety Margins
d=min(gammav-Vv);
if d<0
    d=0;
end

end