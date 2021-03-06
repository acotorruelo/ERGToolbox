function out = rho_poly(this,r,v,x)
%RHO_POLY Navigation Field for the polyhedral/union of polyhedra constraint
%case
%   erg.RHO_POLY(r,v) calculates the Navigation Field for the instance
%   of ERGController erg, with desired reference r and applied reference v.

if this.eta<=0
    error('The value eta must be greater than zero.')
end

xr=this.CL*r;
xv=this.CL*v;
n=size(this.Acl,2);
%Locate current position and reference
r_cur=this.whichPoly(x,v);
r_tgt=this.whichPoly(xr,r);
%Create the graph of the ERGmultiregion
g=graph(this.conn);
%It may happen that x or r are in more than one region. Calculate all
%possibilities
lcur=length(r_cur);
ltgt=length(r_tgt);

paths={};
k=0;
for i=1:lcur
    for j=1:ltgt
        k=k+1;
        paths{k}=shortestpath(g,r_cur(i),r_tgt(j));
    end
end

%Choose the shortest one (all of them are equivalent)
minl=inf;
for i=1:k
    if length(paths{i})<minl
        minl=length(paths{i});
        index=i;
    end
end
path=paths{index};
%Update current and target regions

r_cur=path(1);
r_tgt=path(end);

if r_cur==r_tgt
    ref=r;
else
    next=path(2);
    ref=this.C*this.intsc{r_cur,next}(1:n);
end

%Apply the convex region navigation field
out=(ref-v)/max([norm(ref-v) this.eta]);
end
