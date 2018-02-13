function disp(this)
%DISP Prints the information of the system

%Get system parameters
A=this.A;
B=this.B;
F=this.F;
G=this.G;
P=this.P;
Acl=this.Acl;
Bcl=this.Bcl;
disp(' ')
disp('-----SYSTEM DEFINITION-----')
disp(' ')
if ~isempty(A)
    disp(' ')
    disp('Open loop parameters:')
    disp(' ')
    disp('A=')
    disp(A)
    disp(' ')
    disp('B=')
    disp(B)
    disp(' ')
    disp('F=')
    disp(F)
    disp(' ')
    disp('G=')
    disp(G)
end
disp(' ')
disp('Closed loop parameters:')
disp(' ')
disp('Acl=')
disp(Acl)
disp(' ')
disp('Bcl=')
disp(Bcl)
this.listConstraints

if ~isempty(P)
    disp('-----QUADRATIC LYAPUNOV FUNCTIONS-----')
    disp(' ')
    for i=1:length(P)
        disp(['P' num2str(i) '='])
        disp(P{i})
    end
end

disp(' ')
disp('-----CONSTRAINT MATRICES-----')
disp(' ')
disp('beta_x=')
disp(this.beta_x)
disp(' ')
disp('beta_v=')
disp(this.beta_v)
disp(' ')
disp('h=')
disp(this.h)
end