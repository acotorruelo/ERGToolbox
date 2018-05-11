function out = whichRegion(this,x,v)
%WHICHREGION Check in which region the point x is
%   multiregion.WHICHREGION(x) returns the index (or indices) of the region
%   (or regions) in which state x is.
out=[];
sz=size(x);
if length(sz)>2 || ~any(sz==1)
    error('The point to be checked must be of appropiate dimensions')
elseif sz(1)<sz(2)
    x=x';
elseif sz(1)~=size(this.regions{1}.M,1)
    error('x is not of the appropiate size.')
end

for i=1:length(this.regions)
    if all(this.regions{i}.M*[x;v]<=this.regions{i}.b)
        out=[out;i];
    end
end
end

