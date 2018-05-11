function out = addBetaConstraint(this,beta_x,beta_v,h)
%ADDBETACONSTRAINT Add a constraint to an ERGsys object in beta form
%
%   sys=sys.ADDBETACONSTRAING(beta_x,beta_v,h) adds a constraint to the
%   ERGsys object sys in the form beta_x*x+beta_v*v<=h

if isrow(beta_x)
    beta_x=beta_x';
end
if isrow(beta_v)
    beta_v=beta_v';
end
this.beta_x=[this.beta_x beta_x];
this.beta_v=[this.beta_v beta_v];
this.h=[this.h h];
out=this;
end

