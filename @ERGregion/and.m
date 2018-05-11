function out = and(varargin)
%AND Logic and function override
%
%AND(ERGregion_1,...,ERGregion_n) or ERGregion1 & ERGregion2 outputs an instance
%of ERGregion that represents the intersection of ERGregion1 and ERGregion2
M=[];
b=[];
for i=1:nargin
    if isa(varargin{i},'ERGregion')
        M=[M;varargin{i}.M];
        b=[b;varargin{i}.b];
    else
        error(['Element ' num2str(i) ' is not an instance of the ERGregion class'])
    end
end
out=ERGregion(M,b);
out=out.simplify;
end