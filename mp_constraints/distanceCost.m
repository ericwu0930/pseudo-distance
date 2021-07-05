function h = distanceCost(a,b)
% norm of 2 angle vector
tmp = getMinDistVec(a,b);
h = sum(tmp.^2,2).^(1/2);
end