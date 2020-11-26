function isCol = gjk(v1,v2)
iterCount = 0;
idx = 1;
p1 = mean(v1);
p2 = mean(v2);
d = p1-p2;
if d(1) == 0 && d(2) ==0
    d(1) = 1;
end
simplex = zeros(3,size(v1,2));
simplex(1,:) = support(v1,v2,d);
a = simplex(1,:);
if a*d' <=0
    isCol = 0;
    return;
end
d = -a;
while 1
    iterCount = iterCount+1;
    idx = idx+1;
    simplex(idx,:) = support(v1,v2,d);
    a = simplex(idx,:);
    if a*d'<=0
        isCol = 0;
        return;
    end
    ao = -a;
    if idx < 3
        b = simplex(1,:);
        ab = b-a;
        d = tripleProduct(ab,ao,ab);
        if norm(d)^2 == 0
            d = perpendicular(ab);
        end
        continue;
    end
    
    b = simplex(2,:);
    c = simplex(1,:);
    ab = b-a;
    ac = c-a;
    
    acperp = tripleProduct(ab,ac,ac);
    
    if acperp*ao' >= 0
        d = acperp;
    else
        abperp = tripleProduct(ac,ab,ab);
        if abperp*ao' < 0 
            isCol = 1;
            return;
        end
        simplex(1,:) = simplex(2,:);
        d = abperp;
    end
    
    simplex(2,:) = simplex(3,:);
    idx = idx - 1;
end
isCol = 0;
end

function point = perpendicular(v)
point = [v(2),-v(1)];
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
