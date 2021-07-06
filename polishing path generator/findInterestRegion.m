function [centerID,interestRegion]=findInterestRegion(toolPos,toolNor,workpieceMeanMR)
% find nearby region with largest average positive-MR 
global workpiecePt
global workpieceNormals
global localIDs
global weightOrient; 
toolSurfaceDist=vecnorm(workpiecePt-toolPos,2,2)+weightOrient*acos(min(max(workpieceNormals*toolNor',-1),1)); % tool move cost
weightToolDist=0.001*max(workpieceMeanMR); % the tool should not move too far
weightedReward=workpieceMeanMR-weightToolDist*toolSurfaceDist;
[~,centerID]=max(weightedReward); 
[~,neighborIds]=find(localIDs(centerID,:));
interestRegion=workpiecePt(neighborIds,:);  % in the R-sphere

