function [maxContactDepth]=computeContactDepth(thistoolPose,polishForce)
global toolSamplePoints;
global Disc_R;
global H; 
global workpiecePt;
global gridSize;
[possibleContactIds,~]=find(vecnorm(workpiecePt-thistoolPose(1:3,4)',2,2)<=1.2*sqrt(Disc_R^2+H^2)); % find near points to compute contact force
localPtCloud=(thistoolPose(1:3,1:3)\(workpiecePt(possibleContactIds,:)'-thistoolPose(1:3,4)))';
numNear=12; % find the nearest 10 points in project xy plane
surfaceLocalAlpha=zeros(3,length(toolSamplePoints));
inContact=zeros(length(toolSamplePoints),1);
for k=1:length(toolSamplePoints)
    [distances,list]=sort(sqrt((localPtCloud(:,1)-toolSamplePoints(k,1)).^2+(localPtCloud(:,2)-toolSamplePoints(k,2)).^2));
    inContact(k)=(distances(1)<=1.1*gridSize); % if the test point is in the convex hub
    if inContact(k)
        nearIDs=list(1:numNear);
        X=[ones(numNear,1) localPtCloud(nearIDs,1) localPtCloud(nearIDs,2)];
        Y=localPtCloud(nearIDs,3);
        surfaceLocalAlpha(:,k)=regress(Y,X);   % alpha_local of each point
    else
        surfaceLocalAlpha(:,k)=[0 0 0]';
    end
end
%% initial value
testDepth_1=0; 
testForce_1=computeContactForce(testDepth_1,surfaceLocalAlpha,inContact); 
while(testForce_1==0) % adjust initial depth, if not in contact
    testDepth_1=testDepth_1+0.5; % increase depth
    testForce_1=computeContactForce(testDepth_1,surfaceLocalAlpha,inContact); 
end
while(testForce_1>polishForce) % overlarge contact force
    testDepth_1=testDepth_1-0.5;
    testForce_1=computeContactForce(testDepth_1,surfaceLocalAlpha,inContact); 
end
depth_1=testDepth_1;
depth_2=depth_1+2;
testForce_2=computeContactForce(depth_2,surfaceLocalAlpha,inContact);
residual_1=testForce_1-polishForce; 
residual_2=testForce_2-polishForce;
%% iterative compute contact depth using Newton-Raphson
iterCount=0;
while(abs(residual_2)>5e-1)
    maxContactDepth=depth_2;
    depth_2=depth_2-residual_2/((residual_2-residual_1)/(depth_2-depth_1));
    testForce_2=computeContactForce(depth_2,surfaceLocalAlpha,inContact);
    residual_2=testForce_2-polishForce;
    depth_1=maxContactDepth;
    testForce_1=computeContactForce(depth_1,surfaceLocalAlpha,inContact); 
    residual_1=testForce_1-polishForce;
    iterCount=iterCount+1;
end
maxContactDepth=min(testDepth_1+3,depth_2);  % avoid overlarge contact pressure or overpolishing