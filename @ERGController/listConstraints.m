function listConstraints(this)
%LISTCONSTRAINTS Display a list of the constraints of the ERGController object.

%Retrieve system constraints
l=length(this.h);
disp(' ')
disp('-----LIST OF CONSTRAINTS-----')
disp(' ')
for i=1:l
    if prod(this.beta_v(:,i)==0)
        disp(['#' num2str(i) ' S: [' num2str(this.beta_x(:,i)') ']*x <= ' num2str(this.h(i))])
    elseif all(all(this.beta_v(:,i)'==this.G))
        disp(['#' num2str(i) ' I: u <= ' num2str(this.h(i))])
    elseif all(all(-this.beta_v(:,i)'==this.G))
        disp(['#' num2str(i) ' I: u >= ' num2str(this.h(i))])
    else
        disp(['#' num2str(i) '[' num2str(this.beta_x(:,i)') ']*x + [' num2str(this.beta_v(:,i)') ']*v <=' num2str(this.h(i))])
    end
    disp(' ')
end
end