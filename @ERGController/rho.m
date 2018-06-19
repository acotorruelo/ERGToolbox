%Navigation field for convex constraints
function out = rho(r,v,eta)
%RHO Returns the Navigation Field for a system with a convex set of
%constraints
%   erg.RHO(r,v,eta) calculates the Navigation Field for an instance of
%   ERGController erg, with reference, applied reference and eta parameter
%   value, r, v and eta, respectively.

if eta<=0
    error('The value of eta must be greater than zero.')
else
    out=(v-r)/max([norm(v-r) eta]);
end
end
