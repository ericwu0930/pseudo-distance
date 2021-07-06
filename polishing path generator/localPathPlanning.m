function [pathLocations,toolPoses,toolDwellTimes,thisWorkpieceMR]=localPathPlanning(centerID,polishForce,thisWorkpieceMR)
global gridSize
global localIDs
[~,polishIDs]=find(localIDs(centerID,:));
[feedDirection,regionWidth,~]=findFeedDirection(centerID,thisWorkpieceMR); % shape of the polishing subregion
edgeMargin=gridSize;
left_x=[ ];
left_t={ };
leftToolPose={ };
left_x(1)=-0.01;
[left_t{1},leftToolPose{1},deltaMR_0]=findDwellTime(centerID,polishIDs,feedDirection,left_x,polishForce,thisWorkpieceMR);
deltaMR=deltaMR_0;
thisWorkpieceMR=thisWorkpieceMR+deltaMR;
[newPath,newDwellTime,newToolPose,deltaMR]=findNewPath(centerID,polishIDs,feedDirection,left_x,polishForce,deltaMR,thisWorkpieceMR);
while newPath>=regionWidth(2)+edgeMargin
    left_x=cat(2,newPath,left_x);
    left_t=cat(2,newDwellTime,left_t);
    leftToolPose=cat(2,newToolPose,leftToolPose);
    thisWorkpieceMR=thisWorkpieceMR+deltaMR;
    [newPath,newDwellTime,newToolPose,deltaMR]=findNewPath(centerID,polishIDs,feedDirection,left_x(1),polishForce,deltaMR,thisWorkpieceMR);
end
right_x=0;
right_t=left_t(end);
rightToolPose=leftToolPose(end);
deltaMR=deltaMR_0;
[newPath,newDwellTime,newToolPose,deltaMR]=findNewPath(centerID,polishIDs,feedDirection,right_x(end),polishForce,deltaMR,thisWorkpieceMR);
while newPath<=regionWidth(1)-edgeMargin
    right_x=cat(2,right_x,newPath);
    right_t=cat(2,right_t,newDwellTime);
    rightToolPose=cat(2,rightToolPose,newToolPose);
    thisWorkpieceMR=thisWorkpieceMR+deltaMR;
    [newPath,newDwellTime,newToolPose,deltaMR]=findNewPath(centerID,polishIDs,feedDirection,right_x(end),polishForce,deltaMR,thisWorkpieceMR);
end
pathLocations=[left_x(1:end-1),right_x];
toolDwellTimes=cat(2,left_t(:,1:end-1),right_t);
toolPoses=cat(2,leftToolPose(1:end-1),rightToolPose);
