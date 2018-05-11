function out = isRegional(this)
%ISREGIONAL Check whether or not an instance of ERGsys has an ERGregion or
%ERGmultiregion associated to it.
%   sys.ISREGIONAL returns a boolean value indicating whether or not sys
%   has an ERGregion or ERGmultiregion associated to it.

out=this.info.isRegional;

end

