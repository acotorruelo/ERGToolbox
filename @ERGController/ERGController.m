classdef ERGController < handle
    %ERGController Defines an Explicit Reference Governor (ERG) for a linear system.
    %
    %   VERSION 1.2.1
    %
    %   Constructor:
    %
    %       ERGController(A,B,C,D,F,G) - Creates the ERGController object from the open
    %       loop system matrices A, B, C, D, F (state feedback of the inner
    %       loop) and G (feedforward matrix). A+B*F must be stable.
    %
    %       ERGController(Acl,Bcl,C,D) - Creates the ERGController object from the closed
    %       loop system matrices Acl,Bcl,C and D. Acl must be stable.
    %
    %       ERGController(LTISystem) - Creates the ERGController object
    %       from the closed loop LTISystem object. An LTISystem object represents a
    %       discrete time LTI system, so the sampling time must be provided
    %       for the conversion to continuous time.
    %
    %   ERGController properties:
    %
    %         A - Open loop A matrix
    %         Acl - Closed loop A matrix
    %         B - Open loop B matrix
    %         Bcl - Closed loop B matrix
    %         C - C matrix
    %         D - D matrix
    %         F - Feedback matrix
    %         G - Feedforward matrix
    %
    %   ERGController methods:
    %
    %         Constraint related methods:
    %
    %         addStateConstraint - Add constraint on the state.
    %         addInputConstraint - Add constraint on the input.
    %         addAlphaConstraint - Add generalized constraint in alpha
    %         form.
    %         addBetaConstraint - Add generalized constraint in beta form.
    %         addPolyhedralConstraint - Add polyhedral constraint.
    %         listConstraints - Lists constraints.
    %         removeConstraint - Remove constraint(s).
    %         getPoly - Get polyhedral constraint.
    %
    %         Lyapunov Functions related methods:
    %
    %         getQuadraticLyapunov - Returns the Quadratic Lyapunov
    %         functions of the system.
    %         getQuadraticLyapunov_poly - Returns the Quadratic Lyapunov
    %         functions of the system when subjected to polyhedral
    %         constraints.
    %
    %         Dynamic Safety Margin related methods:
    %
    %         DSM - Computes the Dynamic Safety Margin of the system.
    %
    %         Navigation Field related methods:
    %
    %         NF - Computes the Navigation Field of the system.
    %
    %See also POLYHEDRON, LTISYSTEM
    
    properties (SetAccess = immutable)
        A % Open loop A matrix
        Acl % Closed loop A matrix
        B % Open loop B matrix
        Bcl % Closed loop B matrix
        C % C matrix
        D % D matrix
        F % Feedback matrix
        G % Feedforward matrix
    end
    
    properties (Access = protected)
        beta_x % beta_x constraint matrix
        beta_v % beta_v constraint matrix
        h % h constraint vector
        P % Quadratic Lyapunov functions
        Preg % Quadratic Lyapunov functions for polyhedral constraints
        poly % Polyhedral constraint
        conn % Connection matrix
        intsc % Intersection centers
        delta % Static safety margin
        eta % Smoothing margin
        zeta % Influence margin
        CL %Auxiliary closed loop matrix
    end
    
    methods
        %Constructor
        function this=ERGController(varargin)
            if nargin==1 && isa(varargin{1},'LTISystem')
                %Extract information from the LTISystem object
                sys=ss(varargin{1}.A,varargin{1}.B,varargin{1}.C,varargin{1}.D,varargin{1}.Ts);
                %Convert to continuous time
                sysc=d2c(sys);
                %Check that the system is stable
                if any(real(eig(sysc.A))>0)
                    error('The system is not stable.')
                end
                %Save the matrices
                this.Acl=sysc.A;
                this.Bcl=sysc.B;
                this.C=sysc.C;
                this.D=sysc.D;
            elseif nargin==4
                if size(varargin{1},1)~=size(varargin{1},2)
                    %Check that A is square
                    error('Matrix A must be square.')
                elseif size(varargin{1},1)~=size(varargin{2},1)
                    %Check that A and B are of the correct dimensions
                    error('A and B must have the same number of rows.')
                elseif ~prod(real(eig(varargin{1}))<=0)
                    %Check that the system is stable
                    error('The system is not stable.')
                end
                %Save the matrices
                this.Acl=varargin{1};
                this.Bcl=varargin{2};
                this.C=varargin{3};
                this.D=varargin{4};
            elseif nargin==6
                if size(varargin{1},1)~=size(varargin{1},2)
                    %Check that A is square
                    error('Matrix A must be square.')
                elseif size(varargin{1},1)~=size(varargin{2},1)
                    %Check that A and B are of the correct dimensions
                    error('A and B must have the same number of rows.')
                elseif size(varargin{2},2)~=size(varargin{5},1)
                    %Check that B and F are of the correct dimensions
                    error('The number of columns of B must be the same as the number of rows of F')
                elseif ~prod(real(eig(varargin{1}+varargin{2}*varargin{5}))<=0)
                    %Check that the system is stable
                    error('Closed loop system (A+B*F) is not stable.')
                end
                %Save the matrices
                this.A=varargin{1};
                this.B=varargin{2};
                this.C=varargin{3};
                this.D=varargin{4};
                this.F=varargin{5};
                this.G=varargin{6};
                this.Acl=this.A+this.B*this.F;
                this.Bcl=this.B*this.G;
            end
            this.beta_x=[];
            this.beta_v=[];
            this.h=[];
            
            this.eta=1e-3;
            this.zeta=5e-3;
            this.delta=1e-3;
            
            this.CL=-this.Acl\this.Bcl;
            
            this.P={};
            this.poly=[];
        end
        
        %Add constraint in alpha form
        function this = addAlphaConstraint(this,alpha_x,alpha_u,h,varargin)
            %ADDALPHACONSTRAINT Add a constraint to an ERGController object in alpha form
            %
            %   erg.ADDALPHACONSTRAINT(alpha_x,alpha_u,h) adds a constraint
            %   to the ERGController object erg in the form
            %   alpha_x*x+alpha_u*u<=h and solves the Lyapunov equation
            %   associated to it with the default solver.
            %
            %   erg.ADDALPHACONSTRAINT(alpha_x,alpha_u,h,solver) adds a
            %   constraint to the ERGController object erg in the form
            %   alpha_x*x+alpha_u*u<=h and solves the Lyapunov equation
            %   associated to it with the solver passed as an argument.
            if nargin>3
                solver=varargin{1};
            else
                solver=[];
            end
            if isempty(this.F) || isempty(this.G)
                error('To add an alpha constraint, the ERGController object must be created with the open loop parameters. Try a beta constraint.')
            end
            if isrow(alpha_x)
                alpha_x=alpha_x';
            end
            if isrow(alpha_u)
                alpha_u=alpha_u';
            end
            this.beta_x=[this.beta_x (alpha_x'+alpha_u'*this.F)'];
            this.beta_v=[this.beta_v (alpha_u'*this.G)'];
            this.h=[this.h h];
            
            if ~isempty(solver)
                this.computeIndividualLyap((alpha_x'+alpha_u'*this.F)',solver);
            else
                this.computeIndividualLyap((alpha_x'+alpha_u'*this.F)');
            end
        end
        
        %Add constraint in beta form
        function this = addBetaConstraint(this,beta_x,beta_v,h,varargin)
            %ADDBETACONSTRAINT Add a constraint to an ERGController object in beta form
            %
            %   erg.ADDBETACONSTRAINT(beta_x,beta_v,h) adds a constraint to
            %   the ERGController object erg in the form
            %   beta_x*x+beta_v*v<=h and solves the Lyapunov equation
            %   associated to it with the default solver.
            
            %   erg.ADDBETACONSTRAINT(beta_x,beta_u,h,solver) adds a
            %   constraint to the ERGController object erg in the form
            %   beta_x*x+beta_u*u<=h and solves the Lyapunov equation
            %   associated to it with the solver passed as an argument.
            if nargin>3
                solver=varargin{1};
            else
                solver=[];
            end
            if isrow(beta_x)
                beta_x=beta_x';
            end
            if isrow(beta_v)
                beta_v=beta_v';
            end
            this.beta_x=[this.beta_x beta_x];
            this.beta_v=[this.beta_v beta_v];
            this.h=[this.h h];
            
            if ~isempty(solver)
                this.computeIndividualLyap(beta_x',solver);
            else
                this.computeIndividualLyap(beta_x');
            end
        end
        
        %Add input constraint
        function this = addInputConstraint(this,minmax,b,varargin)
            %ADDINPUTCONSTRAINT Add an input constraint to an ERGController object.
            %
            %   erg.ADDINPUTCONSTRAINT(minmax,b) adds an input constraint
            %   in the form u >= b or u <= b to an instance of
            %   ERGController, erg. The sign of the inequality is set by
            %   the argument minmax, which can only take values 'min' or
            %   'max' for lower and upper bounds, respectively. Afterwards,
            %   it solves the Lyapunov equation associated to the
            %   constraint with the default solver.
            
            %   erg.ADDINPUTCONSTRAINT(minmax,b,solver) adds an input
            %   constraint in the form u >= b or u <= b to an instance of
            %   ERGController, erg. The sign of the inequality is set by
            %   the argument minmax, which can only take values 'min' or
            %   'max' for lower and upper bounds, respectively. Afterwards,
            %   it solves the Lyapunov equation associated to the
            %   constraint with the solver passed as an argument.
            
            if nargin>3
                solver=varargin{1};
            else
                solver=[];
            end
            %Add entries to the beta_x and beta_v matrices depending on the value of ul
            if isempty(this.A)
                error('Input constraints are only allowed if a system is defined by its open loop matrices (A,B,F,G)')
            elseif strcmp(minmax,'max')
                this.beta_x=[this.beta_x this.F'];
                this.beta_v=[this.beta_v this.G'];
                this.h=[this.h b];
            elseif strcmp(minmax,'min')
                this.beta_x=[this.beta_x -this.F'];
                this.beta_v=[this.beta_v -this.G'];
                this.h=[this.h -b];
            else
                error('Parameter ul can only take values max or min')
            end
            if ~isempty(solver)
                this.computeIndividualLyap(this.F',solver);
            else
                this.computeIndividualLyap(this.F');
            end
        end
        
        %Add Polyhedral constraint
        function this = addPolyhedralConstraint(this,poly,varargin)
            %ADDPOLYHEDRALCONSTRAINT Add a polyhedral constraint to an
            %ERGController
            %
            %   erg.ADDPOLYHEDRALCONSTRAINT(poly) adds a polyhedral
            %   constraint to the ERGController and solves the Lyapunov
            %   equation associated to every facet of the Polyhedron
            %   instance, poly, with the default solver.
            %
            %   erg.ADDPOLYHEDRALCONSTRAINT(poly,solver) adds a polyhedral
            %   constraint to the ERGController and solves the Lyapunov
            %   equation associated to every facet of the Polyhedron
            %   instance, poly, with the solver passed as an argument.
            
            if nargin>2
                solver=varargin{1};
            else
                solver=[];
            end
            if ~isa(poly,'Polyhedron')
                error('The argument provided must be an instance of the Polyhedron class.')
            end
            dim=poly.Dim;
            if ~isempty(this.beta_x)
                error('The system already has constraints.')
            elseif size(this.A,2)+size(this.C,1)~=dim
                error(['The dimension of the region being assigned is not the same as the dimension of the system (it must be n+p=' num2str(size(this.A,2)+ size(this.C,1))])
            else
                this.poly=poly;
                %Calculate intersections
                l=length(poly);
                this.conn=zeros(l,l);
                this.intsc=cell(l,l);
                for i=1:l
                    for j=i+1:l
                        %Is there an intersection?
                        int=poly(i)&poly(j);
                        if ~int.isEmptySet
                            %Calculate distance from intersection to the
                            %parent polyhedra
                            v1=poly(i).V;
                            v2=poly(j).V;
                            d=0;
                            n1=size(v1,1);
                            n2=size(v2,1);
                            for p=1:n1
                                for q=1:n2
                                    d=d+norm(v1(p,:)-v2(q,:));
                                end
                            end
                            this.conn(i,j)=d/(n1*n2);
                            this.conn(j,i)=this.conn(i,j);
                            this.intsc{i,j}=mean(int.V)';
                            this.intsc{j,i}=this.intsc{i,j};
                        end
                    end
                end
            end
            
            if ~isempty(solver)
                this.calculateQuadraticLyapunov_poly(solver);
            else
                this.calculateQuadraticLyapunov_poly();
            end
        end
        
        %Add state constraint
        function this = addStateConstraint(this,c,b,varargin)
            %ADDSTATECONSTRAINT Add a state constraint to a ERGController object.
            %
            %   erg=erg.ADDSTATECONSTRAINT(c,b) adds a state constraint in the form
            %   c*x <= b to an instance of ERGController, sys.
            if nargin>3
                solver=varargin{3};
            else
                solver=[];
            end
            n=size(this.Acl,1);
            ny=size(this.C,1);
            %Check that the constraint is of apropriate size
            if size(c,1)~=n
                error('The constraint matrix must have the same number of rows as A')
            end
            %Add the constraints to the beta_x matrix of the ERGControllertem object
            l=length(b);
            this.beta_x=[this.beta_x c];
            this.beta_v=[this.beta_v zeros(ny,l)];
            this.h=[this.h b];
            
            if ~isempty(solver)
                this.computeIndividualLyap(c,solver);
            else
                this.computeIndividualLyap(c);
            end
        end
        
        %Compute DSM
        function d=DSM(this,x,v)
            %DSM Returns the Dynamic Safety Margin of the ERGController for
            %the current state and applied reference
            %
            %erg.DSM(x,v) computes the Dynamic Safety Margin for the
            %ERGController erg with state x and applied reference v
            if ~isempty(this.poly)
                d=this.DSM_Lyap_poly(x,v);
            else
                d=this.DSM_Lyap(x,v);
            end
        end
        
        %Get Quadratic Lyapunov functions
        function P = getQuadraticLyapunov(this)
            %GETQUADRATICLYAPUNOV Returns the calculated Lyapunov functions for the
            %system
            %
            %   erg.GETQUADRATICLYAPUNOV Returns the Lyapunov functions associated to
            %   every constraint of the ERGController object erg.
            
            P=this.P;
        end
        
        %Get Quadratic Lyapunov functions for polyhedral constraints
        function Preg = getQuadraticLyapunov_poly(this)
            %GETQUADRATICREGIONALLYAPUNOV Returns the calculated Lyapunov functions for
            %the ERGmultiregion of the system
            %
            %   erg.GETREGIONALQUADRATICLYAPUNOV Returns the Lyapunov functions
            %   associated to every Polyhedron of the union of polyhedra of
            %   the ERGController object erg.
            
            Preg=this.Preg;
        end
        
        %Get associated region
        function out=getPoly(this)
            %GETREGION Returns the ERGregion or ERGmultiregion of an instance of
            %ERGController.
            %   erg.GETREGION returns the ERGregion or ERGmultiregion of erg.
            out=this.poly;
        end
        
        %Remove constraint
        function this = removeConstraint(this,n)
            %REMOVECONSTRAINT Removes specified constraint(s) from an ERGController object.
            %
            %   erg.REMOVECONSTRAINT(n) removes the constraint(s) specified by the
            %   vector of indices n.
            nc=size(this.beta_x,2);
            if all(n<1) || any(isnan(n)) || any(isempty(n)) || any(isinf(n)) || any(floor(n)~=n)
                %If n is not a positive scalar
                error('n must be a strictly positive integer.')
            elseif n>nc
                %If n is greater than the total number fo constraints
                error(['The system has ' num2str(nc) ' constraints.'])
            elseif length(n)~=length(unique(n))
                %If n has any repeated components
                error('There are duplicates in n')
            end
            this.beta_x(:,n)=[];
            this.beta_v(:,n)=[];
            this.h(n)=[];
            Pnew={};
            cont=1;
            for i=1:length(this.P)
                if ~any(i==n)
                    Pnew{cont}=this.P{i};
                end
            end
        end
        
        %Set parameter
        function this = setParameter(this,param,value)
            %SETPARAMETER Changes the value of a given parameter
            %   erg.SETPARAMETER(p,x) changes the value of parameter p to x
            % Argument p must be a string. Its possible values are listed
            % below:
            %
            %   List of parameters and possible values of p:
            %       Static safety margin - 'delta'
            %       Smoothing margin     - 'eta'
            %       Influence margin     - 'zeta'
            switch param
                case 'delta'
                    if value<=0
                        error('Parameter delta must be greater than zero.')
                    else
                        this.delta=value;
                    end
                case 'eta'
                    if value<=0
                        error('Parameter eta must be greater than zero.')
                    else
                        this.eta=value;
                    end
                case 'zeta'
                    if value<=0
                        error('Parameter zera must be greater than zero')
                    else
                        this.zeta=value;
                    end
                otherwise
                    error(['Argument ' param ' not recognized. Type help ERGController.setParameter for a full list of parameters.'])
            end
        end
        
        function out=NF(this,r,v,x)
            %erg.NF Calculates the Navigation Field for the current
            %reference and applied reference
            %
            %erg.NF(r,v,x) Calculates the Navigation Field for the
            %desired reference r and applied reference v
            if ~isempty(this.poly)
                out=this.rho_poly(r,v,x);
            else
                out=this.rho(r,v);
            end
        end
        
        % Methods defined outside of this file
        
        %Display information
        display(this)
        
        %List constraints
        listConstraints(this)
        
        %Locate (x,v)
        out = whichPoly(this,x,v)
    end
    
    methods (Access=private)
        %Calculate Quadratic Regional Lyapunov functions
        this = calculateQuadraticLyapunov_poly(this,varargin)
        
        %Compute the Lyapunov Function associated to a single constraint
        computeIndividualLyap(this,beta_x,varargin)
        
        %Calculate the Dynamic Safety Margin
        d = DSM_Lyap(this,x,v)
        
        %Calculate the Dynamic Safety Margin for polyhedral constraints
        d = DSM_Lyap_poly(this,x,v)
        
        %Navigation field for convex constraints
        out = rho(this,r,v)
        
        %Navigation field for polyhedral constraints
        out = rho_poly(this,r,v,x)
    end
end