function out = whichPoly(this,x,v)
%WHICHPOLY Check in which polyhedron the point (x,v) is
%
%   erg.WHICHPOLY(x,v) returns the index (or indices) of the
%   polyhedron (or polyhedra) in which the extended state (x,v)
%   is.
out=[];
sz=size(x);
dim=this.poly.Dim;
if length(sz)>2 || ~any(sz==1)
    error('The point to be checked must be of appropiate dimensions')
elseif sz(1)<sz(2)
    x=x';
elseif sz(1)+length(v)~=dim
    error('x is not of the appropiate size.')
end

for i=1:length(this.poly)
    if all(this.poly(i).H(:,1:end-1)*[x;v]<=this.poly(i).H(:,end))
        out=[out;i];
    end
end
end