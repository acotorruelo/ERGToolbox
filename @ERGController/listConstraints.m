function listConstraints(this)
%LISTCONSTRAINTS Display a list of the constraints of the ERGController object.

%Retrieve system constraints
l=length(this.h);
m=size(this.B,2);
disp(' ')
disp('-----LIST OF CONSTRAINTS-----')
disp(' ')
for i=1:l
    %Get input index
    selgt=sum(kron(ones(1,m),this.beta_v(:,i))+this.G',1);
    sellt=sum(kron(ones(1,m),this.beta_v(:,i))-this.G',1);
    indgt=find(selgt==0,1);
    indlt=find(sellt==0,1);
    if prod(this.beta_v(:,i)==0)
        disp(['#' num2str(i) ' S: ' mat2str(this.beta_x(:,i)') '*x <= ' num2str(this.h(i))])
    elseif ~isempty(indlt)
        if m>1
            disp(['#' num2str(i) ' I: u_' num2str(indlt) ' <= ' num2str(this.h(i))])
        else
            disp(['#' num2str(i) ' I: u <= ' num2str(this.h(i))])
        end
    elseif ~isempty(indgt)
        if m>1
            disp(['#' num2str(i) ' I: u_' num2str(indgt) ' >= ' num2str(-this.h(i))])
        else
            disp(['#' num2str(i) ' I: u >= ' num2str(-this.h(i))])
        end
    else
        disp(['#' num2str(i) mat2str(this.beta_x(:,i)') '*x + ' mat2str(this.beta_v(:,i)') '*v <=' num2str(this.h(i))])
    end
    disp(' ')
end
end