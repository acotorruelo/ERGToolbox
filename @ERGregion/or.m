function out = or(varargin)
%OR Overload of the or function. Useful for defining unions of ERGregions.
%   out=or(ERGregion_1,...,ERGregion_n)
%   out - ERGmultiregion object containing the union of ERGregion1 to
%   ERGregion_n
out=ERGmultiregion(varargin{:});
end