function [dwellTimes,toolPoses,deltaWorkpieceMR]=findDwellTime(centerID,polishIDs,feedDirection,path_x,polishForce,thisWorkpieceMR)
global workpiecePt
global workpieceNormals
global Tilt_angle
global Disc_R
global halfPathWidth
global halfPatternWidth
global minPathLength
global holeLength
Normal=workpieceNormals(centerID,:);
Y_axis=feedDirection;
X_axis=cross(Y_axis,Normal);
Rot=[X_axis' Y_axis' Normal'];
center=workpiecePt(centerID,:);
projWorkpiecePt=(Rot\(workpiecePt-center)')'; 
projLocalPt=projWorkpiecePt(polishIDs,:); % local point cloud
% find points on the path_x to define sampled tool poses,  
[inPathIds,~]=find(abs(projLocalPt(:,1)-path_x)<halfPathWidth); 
[sortPathPt_y,sortIds]=sort(projLocalPt(inPathIds,2)); % sort according to -y -> +y
inPathIds=inPathIds(sortIds);
inPathIDs=polishIDs(inPathIds);
% find points in the polishing pattern to compute tool dwell times,  
[inPatternIds,~]=find(abs(projLocalPt(:,1)-path_x)<halfPatternWidth); 
% considering holes in path
diffPathLength=sortPathPt_y(2:end)-sortPathPt_y(1:end-1);
[holeStartIds,~]=find(diffPathLength>holeLength); % partition if spacing bigger than 20
if isempty(holeStartIds)
    pathPartitions=[1,length(inPathIds)];
else
    pathPartitions=zeros(length(holeStartIds)+1,2);
    pathPartitions(1,:)=[1,holeStartIds(1)];
    pathPartitions(end,:)=[holeStartIds(end)+1,length(inPathIds)];
    if length(holeStartIds)>=2
        for i=2:length(holeStartIds)
            pathPartitions(i,:)=[holeStartIds(i-1)+1 holeStartIds(i)];
        end
    end
end
% plan tool dwell time on sample path points
sampleDistance=1; % tool sample distance is 1mm
maxFeedVel=50;
minFeedVel=1;
dwellTimes=[];
toolPoses=[];
deltaWorkpieceMR=zeros(length(workpiecePt),1);
for i=1:length(holeStartIds)+1
    pathLength=sortPathPt_y(pathPartitions(i,2))-sortPathPt_y(pathPartitions(i,1));
    if pathLength>=minPathLength   % a polishing path should not be shorter than 20
        sampleNumber=ceil(pathLength/sampleDistance);
        sampleTool_xy=[path_x*ones(sampleNumber,1) flip(sortPathPt_y(pathPartitions(i,2))-sampleDistance*(0:sampleNumber-1))'];
        toolPose=zeros(4,4,sampleNumber);
        for j=1:sampleNumber
            for k=1:numel(inPathIds) % find adjacent workpiece points to calculate tool normal and position
                if sortPathPt_y(k)>=sampleTool_xy(j,2)
                    up_ID=inPathIDs(k);
                    down_ID=inPathIDs(k-1);
                    break
                end
            end
            normal=workpieceNormals(up_ID,:)/2+workpieceNormals(down_ID,:)/2; normal=normal/norm(normal);
            x_axis=cross(feedDirection,normal)/norm(cross(feedDirection,normal));
            y_axis=cross(normal,x_axis);            
            toolRot=[x_axis' y_axis' normal']*[1 0 0;0 cos(Tilt_angle) sin(Tilt_angle);0 -sin(Tilt_angle) cos(Tilt_angle)];
            projToolPos_z=(projWorkpiecePt(up_ID,3)*(sampleTool_xy(j,2)-sortPathPt_y(k-1))+projWorkpiecePt(down_ID,3)*(sortPathPt_y(k)-sampleTool_xy(j,2)))/(sortPathPt_y(k)-sortPathPt_y(k-1));
            toolPos=Rot*[sampleTool_xy(j,:) projToolPos_z]'+center';
            toolPose(:,:,j)=[toolRot toolPos;0 0 0 1]*[eye(3) [0 -Disc_R 0]';0 0 0 1];
        end
        localRemovalMatrix=findRemovalMatrix(polishIDs,toolPose,polishForce);
        [test_ids,~]=find((projLocalPt(inPatternIds,2)-sortPathPt_y(pathPartitions(i,1))).*(projLocalPt(inPatternIds,2)-sortPathPt_y(pathPartitions(i,2)))<=0);
        testIds=inPatternIds(test_ids);
        testRemovalMatrix=localRemovalMatrix(testIds,:); 
        timeVaryWeight=150;   % consider t change 
        timeDiffMatrix=[diag(ones(1,sampleNumber-1))+diag(-1*ones(1,sampleNumber-2),1) [zeros(sampleNumber-2,1);-1]];
        C=[testRemovalMatrix;timeVaryWeight*timeDiffMatrix];        
        d=[thisWorkpieceMR(polishIDs(testIds)).*patternKernel((projLocalPt(testIds,1)-path_x)/halfPatternWidth);zeros(sampleNumber-1,1)];
        min_t=(sampleDistance/maxFeedVel)*ones(sampleNumber,1);
        max_t=(sampleDistance/minFeedVel)*ones(sampleNumber,1);
        [dwellTime,~]=lsqlin(C,d,[],[],[],[],min_t,max_t); 
        dwellTimes=cat(1,dwellTimes,dwellTime);
        toolPoses=cat(3,toolPoses,toolPose);
        localMR=localRemovalMatrix*dwellTime;
        deltaWorkpieceMR(polishIDs)=deltaWorkpieceMR(polishIDs)-localMR; % update workpieceMR
    end
end