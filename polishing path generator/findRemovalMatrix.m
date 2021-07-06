function [localRemovalMatrix]=findRemovalMatrix(polishRegionIDs,toolPose,polishForce)
global workpiecePt
global E;  % Mpa %modulus /MPa
global Beta;  %power index
global H; % mm %disc thickness /m
global K;
global Omega;
global Disc_R
testWorkpiecePoints=workpiecePt(polishRegionIDs,:);
[~,~,numOfPoses]=size(toolPose);
localRemovalMatrix=zeros(length(polishRegionIDs),numOfPoses);
for j=1:numOfPoses
    thistoolPose=toolPose(:,:,j);
    contactDepth=computeContactDepth(thistoolPose,polishForce);
    toolContactPoses=thistoolPose*[eye(3) [0 0 -contactDepth]';0 0 0 1];
    toolCenter=toolContactPoses(1:3,4)'; 
    discAxis=toolContactPoses(1:3,3)';
    testContactDepths=(testWorkpiecePoints-toolCenter)*discAxis';  % the contact depths of test points
    testCenterDists=sqrt(vecnorm(testWorkpiecePoints-toolCenter,2,2).^2-testContactDepths.^2); % the center dists of test points
    localRemovalMatrix(:,j)=K*E*(testContactDepths/H).^Beta*Omega.*testCenterDists.*(testContactDepths>=0).*(testContactDepths<=H).*(testCenterDists<=Disc_R+0.1);
end