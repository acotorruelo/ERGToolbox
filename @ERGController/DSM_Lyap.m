function d = DSM_Lyap(this,x,v)
%DELTA Calculate the Dynamic Safety Margin.
%
%   d=erg.DELTA(x,v) calculates the Dynamic Safety Margin for the instance
%   of ERGController, sys, for a given state x and applied reference v. For this
%   to be computed, the Lyapunov functions of the system must be calculated
%   first with the method calculateQuadraticLyapunov().

%Calculate the steady state for v
xv=this.CL*v;


nc=size(this.beta_x,2);
gammav=zeros(1,nc);
Vv=gammav;

%Calculate gamma and V for every constraint
for i=1:nc
    gammav(i)=(this.beta_x(:,i)'*xv+this.beta_v(:,i)'*v-this.h(i))^2/(this.beta_x(:,i)'*(this.P{i}\this.beta_x(:,i)));
    Vv(i)=(x-xv)'*this.P{i}*(x-xv);
end

%Return the minimum value of the Dynamic Safety Margins
d=min(gammav-Vv);
if d<0
    d=0;
end

end