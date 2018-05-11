classdef ERGregion
%ERGREGION Defines a polyhedric region of constraints for an ERGsys to
%operate in.
%
%   VERSION 0.5
%
%   Constructor:
%       ERGREGION(M,b) - Creates the ERGregion object from the polyhedric
%       region M*x<=b.
%
%   Methods:
%        and(varargin)
%        
%        or(varargin)
%        
%        out=simplify(this)

    properties (SetAccess = immutable)
        M
        b
        V
    end
    
    methods
        %Constructor
        function this=ERGregion(M,b)
            if size(M,1)~=size(b,1)
                error('M and b matrices must have the same number of rows')
            else 
                this.M=M;
                this.b=b;
                % Calculate vertices
                p=Polyhedron(M,b);
                p=p.minVRep;
                this.V=p.V;
                
            end
        end
        
        %And override
        out=and(varargin)
        
        %Or override
        out=or(varargin)
        
        %Simplify
        out=simplify(this)
        
        %Disp overload
        disp(this)
    end
end

