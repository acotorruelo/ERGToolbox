function out = calculateQuadraticLyapunov(this)
%CALCULATEQUADRATICLYAPUNOV Calculate Lyapunov functions for every
%constraint in the ERGsys object
%   Description:
%       Calculate Lyapunov function for every constraint in the ERGsys
%       object. This function returns a cell object with as many entries as
%       constraints in the ERGsys object. If the ERGsystem object has no
%       constraints, it calculates the largest Lyapunov function 
%       A'P + P*A <0
%
%   Parameters:
%       this    ERGsys of which the Lyapunov functions are to be calculated

%Retrieve system parameters
beta_x=this.beta_x;
%Calculate the closed loop A
A=this.Acl;
h=this.h;
l=length(h);
n=size(A,2);
%Create the output variable
if ~l
    Pcell=cell(1);
else
    Pcell=cell(1,l);
end

% LMI problem definition

P=sdpvar(n,n,'symmetric');
fcn=trace(P);
epsilon=1e-4;
if ~l
    %If there are no constraints, calculate the solution to the Lyapunov
    %equation
    cons=[
        A'*P+P*A<=-epsilon
        P>=epsilon;
        ];
    
    optimize(cons,fcn);
    Pcell{1}=double(P);
else
    %If there are constraints, calculate the Lyapunov function for every
    %constraint
    for i=1:l
        cons=[
            A'*P+P*A<=-epsilon
            P>=beta_x(:,i)*beta_x(:,i)'/norm(beta_x(:,i))^2;
            ];
        optimize(cons,fcn);
        %Store the P matrix
        Pcell{i}=double(P);
        if ~all(eig(Pcell{i})>0)
            %If not semidefinite, terminate execution
            error(['Matrix P for constraint ' num2str(i) ' is not positive semidefinite.'])
        end
    end
end
this.P=Pcell;
out=this;
end