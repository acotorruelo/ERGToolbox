function display(this)
%DISP Prints the information of the system
if isempty(this.Acl)
    fprintf('Empty ERG controller\n')
else
    %Get system parameters
    disp(' ')
    disp('-----SYSTEM DEFINITION-----')
    disp(' ')
    if ~isempty(this.A)
        disp(' ')
        disp('Open loop parameters:')
        disp(' ')
        disp('A=')
        disp(this.A)
        disp(' ')
        disp('B=')
        disp(this.B)
        disp(' ')
        disp('F=')
        disp(this.F)
        disp(' ')
        disp('G=')
        disp(this.G)
    end
    disp(' ')
    disp('Closed loop parameters:')
    disp(' ')
    disp('Acl=')
    disp(this.Acl)
    disp(' ')
    disp('Bcl=')
    disp(this.Bcl)
    if ~isempty(this.h)
        this.listConstraints
    end
    if ~isempty(this.P)
        disp('-----QUADRATIC LYAPUNOV FUNCTIONS-----')
        disp(' ')
        for i=1:length(this.P)
            disp(['P' num2str(i) '='])
            disp(this.P{i})
        end
    end
    if ~isempty(this.h)
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
    if ~isempty(this.poly)
        disp('-----POLYHEDRON-----')
        disp(this.poly)
    end
end
end