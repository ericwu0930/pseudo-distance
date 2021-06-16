function P = getControlPoints(path,u,U,p)
P = [];
n = length(path);
phi = zeros(n,n);
for i = 1:n
    s = findspan(n-1,p,u(i),U);
    phi(i,s+1-p:s+1) = basisfun(s,u(i),p,U);
end
P = ones(n,6);
for i = 1:6
    P(:,i) = inv(phi'*phi)*(phi)'*path(:,i);
end
end