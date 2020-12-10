function isCols = gjk3d(v1,v2)
% inspired by https://caseymuratori.com/blog_0003
% v1 vertex set 1
% v2 vertex set 2
isCols = 0;
idx = 1;
p1 = mean(v1);
p2 = mean(v2);
D = p1-p2;
if D(1) == 0 && D(2) == 0 && D(3) == 0
    D(1) = 1;
end
simplex = zeros(4,size(v1,2));
simplex(1,:) = support(v1,v2,D);
a = simplex(1,:);
if a*D' <=0
    isCols = 0; 
    return;
end
D = -a;
while 1
    a = support(v1,v2,D);
    if a*D'<=0
        isCols = 0;
        return;
    end
    idx = idx+1;
    simplex(idx,:)=a;
    [isCols,simplex,D,idx] = doSimplex(simplex,idx);
    if isCols == 1
        return;
    end
end
end

function [isCols,simplex,D,idx] = doSimplex(simplex,idx)
if idx == 2
    [isCols,simplex,D,idx] = examLine(simplex,idx);
elseif idx == 3
    [isCols,simplex,D,idx] = examTriangle(simplex,idx);
elseif idx == 4
    [isCols,simplex,D,idx] = examTetrahedron(simplex,idx);
end
end

function [isCols,simplex,D,idx] = examLine(simplex,idx)
isCols = 0;
a = simplex(2,:);
b = simplex(1,:);
ab = b-a;
ao = -a;
if ab*ao'>0
    D = tripleProduct(ab,ao,ab);
else
    idx = 1;
    simplex(:,1)=a;
    D = ao;
end
end

function [isCols,simplex,D,idx] = examTriangle(simplex,idx)
isCols = 0;
a = simplex(3,:);
b = simplex(2,:);
c = simplex(1,:);
ab = b-a;
ac = c-a;
abc = cross(ab,ac);
ao = -a;
if cross(abc,ac)*ao'>=0
    if ac*ao'>=0
        idx = 2;
        simplex(2,:) = a;
        D = tripleProduct(ac,ao,ac);
    else
        if ab*ao'>=0
            idx = 2;
            simplex(1,:) = b;
            simplex(2,:) = a;
            D = tripleProduct(ab,ao,ab);
        else
            idx = 1;
            simplex(1,:) = a;
            D = ao;
        end
    end
else
    if cross(ab,abc)*ao'>=0
        if ab*ao'>=0
            idx = 2;
            simplex(1,:) = b;
            simplex(2,:) = a;
            D = tripleProduct(ab,ao,ab);
        else
            idx = 1;
            simplex(1,:) = a;
            D = ao;
        end
    else
        if abc*ao'>=0
            D = abc;
        else
            D = -abc;
            tmp = simplex(2,:);
            simplex(2,:)=simplex(1,:);
            simplex(1,:)=tmp;
        end
    end
end
end

function [isCols,simplex,D,idx] = examTetrahedron(simplex,idx)
isCols = 0;
a = simplex(4,:);
b = simplex(3,:);
c = simplex(2,:);
d = simplex(1,:);
ab = b-a;
ac = c-a;
ad = d-a;
ao = -a;
abc = cross(ab,ac);
acd = cross(ac,ad);
adb = cross(ad,ab);
if abc*ao'>=0
    idx = 3;
    simplex(1,:) = c;
    simplex(2,:) = b;
    simplex(3,:) = a;
    D = abc;
else
    acd = cross(ac,ad);
    if acd*ao'>0
        idx = 3;
        simplex(3,:) = a;
        D = acd;
    elseif adb*ao'>0
        idx = 3;
        simplex(2,:) = b;
        simplex(3,:) = a;
        D = adb;
    else
        D=[];
        isCols = 1;
    end
end
end

function point = tripleProduct(v1,v2,v3)
point = v1*v3'*v2-v2*v3'*v1;
end

function point = support(v1,v2,d)
idx1 = indexOfFurthestPoint(v1,d);
idx2 = indexOfFurthestPoint(v2,-d);
point = v1(idx1,:)-v2(idx2,:);
end

function idx = indexOfFurthestPoint(v1,d)
% Get furthest vertex along a certain direction
d_=repmat(d,size(v1,1),1);
product = sum(v1.*d_,2);
[~,idx]=max(product);
end