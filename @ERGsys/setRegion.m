function out= setRegion(this,region)
%SETREGION Associate an instance of ERGregion or ERGmultiregion to an
%instance of ERGsys.
%
%   sys=sys.SETREGION(region) sets the ERGregion or ERGmultiregion object
%   region to the ERGsys object sys.
if isa(region,'ERGregion')
    dimreg=size(region.M,2);
elseif isa(region,'ERGmultiregion')
    dimreg=size(region.regions{1}.M,2);
end
if ~(isa(region,'ERGregion')||isa(region,'ERGmultiregion'))
    error('The argument provided must be an instance of ERGregion or ERGmultiregion')
elseif ~isempty(this.beta_x)
    error('The system already has constraints')
elseif size(this.A,2)+ size(this.C,1)~=dimreg
   error(['The dimension of the region being assigned is not the same as the dimension of the system (it must be n+p=' num2str(size(this.A,2)+ size(this.C,1))])
else
    this.region=region;
    this.info.isRegional=1;
    out=this;
end