classdef ERGsys
    %ERGSYS Defines a linear system controlled by an Explicit Reference
    %Governor (ERG). 
    %
    %   VERSION 0.5
    %
    %   Constructor:
    %       ERGsys(A,B,F,G) - Creates the ERGsys object from the matrices A
    %       (state matrix), B (input matrix), F (feedback of the inner
    %       loop) and G (feedforward matrix). A+B*F must be stable.
    %
    %   Methods:
    %         addStateConstraint(this,c,b)
    %         
    %         addInputConstraint(this,ul,b)
    %         
    %         listConstraints()
    %         
    %         removeConstraint(this,n)
    %         
    %         calculateQuadraticLyapunov()
    %         
    %         getQuadraticLyapunov()
    %         
    %         delta(this,x,v)
    %
    %         disp()
    
    properties (SetAccess = immutable)
        A
        Acl
        B
        Bcl
        F
        G
    end
    
    properties (Access = protected)
        beta_x
        beta_v
        h
        P
    end
    
    methods
        %Constructor
        function this=ERGsys(A,B,varargin)
            if nargin~=4 && nargin~=2
                %Check that the number of arguments is proper
                error('Arguments needed: matrices A and B of the closed loop system or A, B, F and G of the open loop system')
            elseif nargin==4
                %Assign the value of the attributes
                this.A=A;
                this.B=B;
                this.F=varargin{1};
                this.G=varargin{2};
                this.Acl=A+B*this.F;
                this.Bcl=B*this.G;
            elseif nargin==2
                this.Acl=A;
                this.Bcl=B;
            end
            if size(A,1)~=size(A,2)
                %Check that A is square
                error('Matrix A must be square')
            elseif size(A,1)~=size(B,1)
                %Check that A and B are of the correct dimensions
                error('A and B must have the same number of rows')
            elseif nargin==4&&(size(B,2)~=size(this.F,1))
                %Check that B and F are of the correct dimensions
                error('The number of columns of B must be the same as the number of rows of F')
            elseif ~prod(real(eig(this.Acl))<0)
                %Check that the system is stable
                error('Closed loop system (A+B*K) is not stable.')
            end
            this.beta_x=[];
            this.beta_v=[];
            this.h=[];
        end
        
        %Add state constraint
        out=addStateConstraint(this,c,b)
        
        %Add input constraint
        out=addInputConstraint(this,ul,b)
        
        %List constraints
        listConstraints(this)
        
        %Remove constraint
        out=removeConstraint(this,n)
        
        %Calculate Quadratic Lyapunov functions
        out=calculateQuadraticLyapunov(this)
        
        %Return Quadratic Lyapunov functions
        out=getQuadraticLyapunov(this)
        
        %Calculate Dynamic Safety Margin
        g=delta(this,x,v)
        
        %Display the contents of the object
        disp(this)
    end
end