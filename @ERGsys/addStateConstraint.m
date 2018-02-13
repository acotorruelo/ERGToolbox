function out = addStateConstraint(this,c,b)
%ADDSTATECONSTRAINT Add a state constraint to a ERGsys object 
%   Description:
%       Add an input constraint to the ERGsys object in the form 
%       c*x <= b
%
%   Parameters:
%       this    ERGsys object to which the constraint is to be added
%
%       c       c matrix
%
%       b       b matrix

n=size(this.Acl,1);
ny=size(this.Bcl,2);
%Check that the constraint is of apropriate size
if size(c,1)~=n
    error('The constraint matrix must have the same number of rows as A')
end
%Add the constraints to the beta_x matrix of the ERGSystem object
l=length(b);
this.beta_x=[this.beta_x c];
this.beta_v=[this.beta_v zeros(ny,l)];
this.h=[this.h b];
out=this;
end

