function [DT,bdidxn] = MinkowskiSum(A,B)
% get the minkowski sum of two polyhedra expressed by two vertices
% A,B : vertex set, n*m matrix n is the dimension of the vector space
[VecDim,m1] = size(A);
m2 = size(B,2);
X = zeros(VecDim,m1*m2);
for i=1:m2
    X(:,(i-1)*m1+1:i*m1) = A + B(:,i);
end
X = unique(X','rows');
% DT = delaunayTriangulation(X);
% DIdx = convexHull(Tes);
% DT = Tes.Points(DIdx,:);
bdidx = convhulln(X);
bdidxn = zeros(length(bdidx),1);
ubdidx = unique(bdidx(:));
for i = 1:length(ubdidx)
    idxtmp = find(bdidx == ubdidx(i));
    bdidxn(idxtmp) = i*ones(size(idxtmp));
end
bdidxn = reshape(bdidxn,size(bdidx));
DT = X(ubdidx,:);
end