function h = distanceCost(a,b)
tmp = getMinDistVec(a,b);
h = sum(tmp.^2,2).^(1/2);
end