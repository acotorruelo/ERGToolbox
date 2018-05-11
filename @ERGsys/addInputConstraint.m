function out = addInputConstraint(this,ul,b)
%ADDINPUTCONSTRAINT Add an input constraint to an ERGsys object.
%
%   sys=sys.ADDINPUTCONSTRAINT(ul,b) adds an input constraint  in the form 
%   u >= b or u <= b to an instance of ERGsys, sys. The sign of the
%   inequality is set by the argument ul, which can only take values 'u' or
%   'l' for upper and lower bounds, respectively.

%Add entries to the beta_x and beta_v matrices depending on the value of ul
if isempty(this.A)
    error('Input constraints are only allowed if a system is defined by its open loop matrices (A,B,F,G)')
elseif strcmp(ul,'u')
    this.beta_x=[this.beta_x this.F'];
    this.beta_v=[this.beta_v this.G'];
elseif strcmp(ul,'l')    
    this.beta_x=[this.beta_x -this.F'];
    this.beta_v=[this.beta_v -this.G'];
else
    error('Parameter ul can only take values u or l')
end
this.h=[this.h b];
out=this;
end

