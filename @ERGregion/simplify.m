function out = simplify(this)
%SIMPLIFY Deletes any redundant constraints from an ERGregion object
%
%region=region.SIMPLIFY checks every constraint and outputs a simplified
%version of the ERGregion instance, region.
options=optimset('Display','none');
M=this.M;
b=this.b;
nc=length(b);
flag=zeros(1,nc);
for i=1:nc
    %For every constraint
    %Flip the i-th constraint
    Mtemp=M;
    Mtemp(i,:)=-Mtemp(i,:);
    btemp=b;
    btemp(i)=-btemp(i);
    [~,~,flag(i)]=linprog([],Mtemp,btemp,[],[],[],[],[],options);
end
out=ERGregion(M(flag>0,:),b(flag>0,:));
end

