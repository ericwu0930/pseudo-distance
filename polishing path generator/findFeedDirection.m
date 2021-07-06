function [feedDirection,regionWidth,regionLength]=findFeedDirection(centerID,thisWorkpieceMR)
global workpiecePt
global workpieceNormals
global localIDs
global halfPatternWidth
global halfPathWidth
global minPathLength;
global holeLength;
%% maximum patternWidth MR
[~,interestIDs]=find(localIDs(centerID,:));
interestRegion=workpiecePt(interestIDs,:);
center=workpiecePt(centerID,:);
normal=workpieceNormals(centerID,:);
x_axis=[1 0 0];   %the x-axis should point to the farest point
Tv=cross(normal,x_axis); 
Tu=cross(Tv,normal); 
Tu=Tu/norm(Tu); Tv=Tv/norm(Tv);
maxMR=0;
opti_i=0;
testAngles=30;
for i=0:testAngles % 40 test directions
    testAngle=i*pi/testAngles;
    Rot=[Tu',Tv',normal']*[cos(testAngle) -sin(testAngle) 0;sin(testAngle) cos(testAngle) 0;0 0 1];
    projInterestRegion=(Rot\(interestRegion-center)')';
    [inPathIds,~]=find(abs(projInterestRegion(:,1))<halfPathWidth); 
    [sortPathPt_y,sortIds]=sort(projInterestRegion(inPathIds,2)); % sort according to -y -> +y
    inPathIds=inPathIds(sortIds);
    diffPathLength=sortPathPt_y(2:end)-sortPathPt_y(1:end-1);
    [holeStartIds,~]=find(diffPathLength>holeLength); % partition if spacing bigger than 20
    if isempty(holeStartIds)
        pathPartitions=[1,length(inPathIds)];
    else
        pathPartitions=zeros(length(holeStartIds)+1,2);
        pathPartitions(1,:)=[1,holeStartIds(1)];
        pathPartitions(end,:)=[holeStartIds(end)+1,length(inPathIds)];
        if length(holeStartIds)>=2
            for k=2:length(holeStartIds)
                pathPartitions(k,:)=[holeStartIds(k-1)+1 holeStartIds(k)];
            end
        end
    end
    pathEffective=max(sortPathPt_y(pathPartitions(:,2))-sortPathPt_y(pathPartitions(:,1)))>minPathLength;
    [inPatternIds,~]=find(abs(projInterestRegion(:,1))<halfPatternWidth); 
    patternWidthMR=thisWorkpieceMR(interestIDs(inPatternIds)).*patternKernel(projInterestRegion(inPatternIds,1)/halfPatternWidth);
    maxMR1=sum(patternWidthMR); 
    if pathEffective && maxMR1>maxMR 
      maxMR=maxMR1;
      opti_i=i;
    end
end
optiAngle=opti_i*pi/testAngles;
Rot=[Tu',Tv',normal']*[cos(optiAngle) -sin(optiAngle) 0;sin(optiAngle) cos(optiAngle) 0;0 0 1];
projInterestRegion=(Rot\(interestRegion-center)')';
feedDirection=Rot(1:3,2)'; % along longer side
regionWidth=[max(projInterestRegion(:,1)),min(projInterestRegion(:,1))];
regionLength=[max(projInterestRegion(:,2)),min(projInterestRegion(:,2))];
%% minimum area bounding box
% for i=0:testAngles % 40 test directions
%     testAngle=i*pi/testAngles;
%     Rot=[Tu',Tv',normal']*[cos(testAngle) -sin(testAngle) 0;sin(testAngle) cos(testAngle) 0;0 0 1];
%     projInterestRegion=(Rot\(interestRegion-center)')';
%     area1=(max(projInterestRegion(:,1))-min(projInterestRegion(:,1)))*(max(projInterestRegion(:,2))-min(projInterestRegion(:,2)));
%     if area1<area
%       area=area1;
%       opti_i=i;
%     end
% end
% optiAngle=opti_i*pi/testAngles;
% Rot=[Tu',Tv',normal']*[cos(optiAngle) -sin(optiAngle) 0;sin(optiAngle) cos(optiAngle) 0;0 0 1];
% projInterestRegion=(Rot\(interestRegion-center)')';
% if max(projInterestRegion(:,1))-min(projInterestRegion(:,1))>max(projInterestRegion(:,2))-min(projInterestRegion(:,2))
%     feedDirection=Rot(1:3,1)'; % along longer side
%     regionWidth=[-min(projInterestRegion(:,2)),-max(projInterestRegion(:,2))];
%     regionLength=[max(projInterestRegion(:,1)),min(projInterestRegion(:,1))];
% %     feedDirection=Rot(1:3,2)'; % along short side
% %     regionWidth=[max(projInterestRegion(:,1)),min(projInterestRegion(:,1))];
% %     regionLength=[max(projInterestRegion(:,2)),min(projInterestRegion(:,2))];
% else
%     feedDirection=Rot(1:3,2)'; % along longer side
%     regionWidth=[max(projInterestRegion(:,1)),min(projInterestRegion(:,1))];
%     regionLength=[max(projInterestRegion(:,2)),min(projInterestRegion(:,2))];
% %     feedDirection=Rot(1:3,1)'; % along short side
% %     regionWidth=[-min(projInterestRegion(:,2)),-max(projInterestRegion(:,2))];
% %     regionLength=[max(projInterestRegion(:,1)),max(projInterestRegion(:,1))];
% end