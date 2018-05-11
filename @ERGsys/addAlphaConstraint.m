function out = addAlphaConstraint(this,alpha_x,alpha_u,h)
%ADDALPHACONSTRAINT Add a constraint to an ERGsys object in alpha form
%
%   sys=sys.ADDALPHACONSTRAING(alpha_x,alpha_u,h) adds a constraint to the
%   ERGsys object sys in the form alpha_x*x+alpha_u*u<=h

if isempty(this.F) || isempty(this.G)
    error('To add an alpha constraint, the ERGsys object must be created with the open loop parameters. Try a beta constraint.')
end
if isrow(alpha_x)
    alpha_x=alpha_x';
end
if isrow(alpha_u)
    alpha_u=alpha_u';
end
this.beta_x=[this.beta_x (alpha_x'+alpha_u'*this.F)'];
this.beta_v=[this.beta_v (alpha_u'*this.G)'];
this.h=[thish h];
out=this;
end

