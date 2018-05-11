classdef ERGsys
    %ERGSYS Defines a linear system controlled by an Explicit Reference
    %Governor (ERG). 
    %
    %   VERSION 0.5
    %
    %   Constructor:
    %       ERGSYS(A,B,C,D,F,G) - Creates the ERGsys object from the open
    %       loop system matrices A, B, C, D, F (state feedback of the inner
    %       loop) and G (feedforward matrix). A+B*F must be stable.
    %
    %       ERGSYS(Acl,Bcl,C,D) - Creates the ERGsys object from the closed
    %       loop system matrices Acl,Bcl,C and D. Acl must be stable.
    %
    %   Methods:
    %         addStateConstraint(c,b)
    %         
    %         addInputConstraint(ul,b)
    %
    %         addAlphaConstraint(alpha_x,alpha_u,h)
    %         
    %         addBetaConstraint(beta_x,beta_v,h)
    %         
    %         listConstraints()
    %         
    %         removeConstraint(n)
    %         
    %         calculateQuadraticLyapunov()
    %         
    %         getQuadraticLyapunov()
    %         
    %         delta(x,v)
    %
    %         disp()
    %
    %         setRegion(region)
    %
    %         calculateQuadraticRegionalLyapunov()
    %
    %         isRegional()
    %
    %         delta_reg(x,v)
    %
    %         getRegion()
        
    properties (SetAccess = immutable)
        A
        Acl
        B
        Bcl
        C
        D
        F
        G
    end
    
    properties (Access = protected)
        beta_x
        beta_v
        h
        P
        Preg
        region
        info
    end
    
    methods
        %Constructor
        function this=ERGsys(A,B,C,D,varargin)
            if nargin~=6 && nargin~=4
                %Check that the number of arguments is proper
                error('Arguments needed: matrices A and B of the closed loop system or A, B, F and G of the open loop system')
            elseif nargin==6
                %Assign the value of the attributes
                this.A=A;
                this.B=B;
                this.C=C;
                this.D=D;
                this.F=varargin{1};
                this.G=varargin{2};
                this.Acl=A+B*this.F;
                this.Bcl=B*this.G;
            elseif nargin==4
                this.Acl=A;
                this.Bcl=B;
                this.C=C;
                this.C=C;
            end
            if size(A,1)~=size(A,2)
                %Check that A is square
                error('Matrix A must be square')
            elseif size(A,1)~=size(B,1)
                %Check that A and B are of the correct dimensions
                error('A and B must have the same number of rows')
            elseif nargin==6 && (size(B,2)~=size(this.F,1))
                %Check that B and F are of the correct dimensions
                error('The number of columns of B must be the same as the number of rows of F')
            elseif ~prod(real(eig(this.Acl))<=0)
                %Check that the system is stable
                error('Closed loop system (A+B*F) is not stable.')
            end
            this.beta_x=[];
            this.beta_v=[];
            this.h=[];
            
            %Create the info structure
            info.hasSaturation=0;
            info.isRegional=0;
            this.info=info;
        end
        
        %Add state constraint
        out=addStateConstraint(this,c,b)
        
        %Add input constraint
        out=addInputConstraint(this,ul,b)
        
        %Add alpha constraint
        out=addAlphaConstraint(alpha_x,alpha_u,h)
        
        %Add beta constraint
        out=addBetaConstraint(beta_x,beta_v,h)
        
        %List constraints
        listConstraints(this)
        
        %Remove constraint
        out=removeConstraint(this,n)
        
        %Calculate Quadratic Lyapunov functions
        out=calculateQuadraticLyapunov(this,varargin)
        
        %Return Quadratic Lyapunov functions
        out=getQuadraticLyapunov(this)
        
        %Calculate Dynamic Safety Margin
        g=delta(this,x,v)
        
        %Display the contents of the object
        disp(this)
        
        %Set a region for the ERG system to operate in
        out=setRegion(this,region)
        
        %Calculate the Regional Lyapunov functions
        out=calculateQuadraticRegionalLyapunov(this,varargin)
        
        %Query for regional
        out=isRegional(this)
        
        %Calculate the regional DSM
        d=delta_reg(this,x,v)
        
        %Return the region
        out=getRegion(this)
    end
end