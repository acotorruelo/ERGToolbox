function out = addInputConstraint(this,ul,b)
%ADDINPUTCONSTRAINT Add an input constraint to an ERGsys object
%   Description:
%       This function adds an input constraint to an ERGsys object, in
%       the form u >= umin or u <= umax
%
%   Parameters:
%       this    ERGsys object to which the constraint is to be added
%
%       ul      Upper or lower bound of the control signal, can only take
%               values 'u' or 'l' for upper and lower bounds, respectively.
%
%       b       Upper or lower bound

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

