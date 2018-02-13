function listConstraints(this)
%LISTCONSTRAINTS Display a list of the constraints of the ERGsys object

%Retrieve system constraints
beta_x=this.beta_x;
beta_v=this.beta_v;
h=this.h;
l=length(h);
disp(' ')
disp('-----LIST OF CONSTRAINTS-----')
disp(' ')
for i=1:l
    if prod(beta_v(i)==0)
        disp(['#' num2str(i) ' S: [' num2str(beta_x(:,i)') ']*x <= ' num2str(h(i))])
    else
        if beta_v(i)==this.G
            disp(['#' num2str(i) ' I: u <= ' num2str(h(i))])
        else
            disp(['#' num2str(i) ' I: u >= ' num2str(h(i))])
        end
    end
end
disp(' ')
end