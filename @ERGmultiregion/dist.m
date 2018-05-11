function d = dist(reg1,reg2)
%DIST Compute the distance between two ERGregions.
%   DIST(reg1,reg2) computes the distance between two instances of
%   ERGregion reg1 and reg2. This distance is computed as the average
%   distance between all vertices.

%% STEP 1: COMPUTE THE INTERSECTION VERTICES
%Calculate all the vertices of reg1 and reg2

v1=reg1.V;
v2=reg2.V;
%% STEP 2: COMPUTE THE AVERAGE DISTANCE BETWEEN VERTICES
d=0;
n1=size(v1,1);
n2=size(v2,1);
for i=1:n1
    for j=1:n2
        d=d+norm(v1(i,:)-v2(j,:));
    end
end
d=d/(n1*n2);
end

