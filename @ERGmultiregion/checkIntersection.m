function out = checkIntersection(reg1,reg2)
%CHECKINTERSECTION Check if two ERGregions have a nonempty intersection.
%   CHECKINTERSECTION(reg1,reg2) checks if instances of ERGregion reg1 and
%   reg2 have a nonempty intersection. It returns 1 if they do, and 0 if
%   they do not.
if ~(isa(reg1,'ERGregion')&&isa(reg2,'ERGregion'))
    error('Both arguments must be instances of ERGregion')
end
opt=optimoptions('linprog','Display','off');
MM=[reg1.M;reg2.M];
bb=[reg1.b;reg2.b];
[~,~,flag]=linprog([],MM,bb,[],[],[],[],opt);
if flag==1
    out=1;
else
    out=0;
end
end