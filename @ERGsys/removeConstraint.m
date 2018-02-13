function out = removeConstraint(this,n)
%REMOVECONSTRAINT Removes specified constraints from the ERGsys object
%   Parameters:
%       this - ERGsys object
%
%       n    - Indices of the constraints to be removed. They can be listed
%              with the listConstraints() method. n can be a vector.
                
this.beta_x(:,n)=[];
this.beta_v(:,n)=[];
this.h(n)=[];
out=this;
end

