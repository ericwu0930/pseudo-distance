%% constant process parameter
global E; E=0.84;  % Mpa %modulus /MPa
global Beta; Beta=0.89; %power index
global H; H=10;  % mm %disc thickness /m
global K; K=0.028;  % 0.0000136*1000 μm/N %material removal coefficient
global Disc_R; Disc_R=40; % mm, tool radius
global Tilt_angle; Tilt_angle=12*pi/180; % constant tilt angle
global Omega; Omega=2*pi*5000/60; % 5000r /min
global toolSamplePoints; toolSamplePoints=[];
global toolWeightArea; toolWeightArea=[]; 
global weightOrient; weightOrient=200;
toolSampleDistance=1;
for r=Disc_R/2:toolSampleDistance:Disc_R
    for theta=2*pi/r:2*pi/r:2*pi
        toolSamplePoints=cat(1,toolSamplePoints,[r*cos(theta) r*sin(theta)]);
        toolWeightArea=cat(1,2*pi*toolSampleDistance,toolWeightArea);
    end
end
%% load workpiece
global gridSize; gridSize=2;
global workpiecePt;
global workpieceNormals;
workpiecePtCloud=pcread('Wheelhub_1.ply'); % with sampling distance 2mm
workpiecePtCloud=pctransform(workpiecePtCloud, rigid3d([-1 0 0; 0 0 1; 0 1 0], [0 0 -60]));
ptIndices=1:workpiecePtCloud.Count;
selectPoints=ptIndices(workpiecePtCloud.Normal(ptIndices,3)>=0.9);
workpiecePtCloudUp=select(workpiecePtCloud,selectPoints);  % select the upper surface want to polish
workpiecePt=double(workpiecePtCloudUp.Location);
workpieceNormals = double(workpiecePtCloud.Normal(selectPoints,:));
ptIndices=1:length(workpiecePt);
upperIDs=ptIndices(vecnorm(workpiecePt(ptIndices,1:2),2,2)<=60); 
sideIDs=ptIndices(vecnorm(workpiecePt(ptIndices,1:2),2,2)>60);
workpieceNormals(upperIDs,:)=ones(length(upperIDs),1)*[0 0 1];
phi1=atan((200-60)/50); % use normals from CAD 
workpieceNormals(sideIDs,:)=[cos(phi1)*workpiecePt(sideIDs,1)./vecnorm(workpiecePt(sideIDs,1:2),2,2) cos(phi1)*workpiecePt(sideIDs,2)./vecnorm(workpiecePt(sideIDs,1:2),2,2) sin(phi1)*ones(length(sideIDs),1)];
workpieceNormals(sideIDs,:)=workpieceNormals(sideIDs,:)./vecnorm(workpieceNormals(sideIDs,:),2,2);
workpiecePtCloudUp.Normal=workpieceNormals;
% pcshow(workpiecePtCloudUp)
%% local point: distances
global interestRadius;
interestRadius=80;
global mutualDists
mutualDists=m_pdist(single(workpiecePt),single(workpieceNormals));
global localIDs
localIDs=sparse(mutualDists-interestRadius<=0); % euclidean distance smaller than interest radius
%% global workpiece
polishForce=12; % constant polish force
toolPos=[0 0 200];
toolNor=[0 0 1]; % initial tool configurationcenterIDs=[]; % save states
subregionCount=0; % number of subregions
centerIDs=[];
localPath_x={}; % local path x
sampleToolPoses={}; % tool sample poses, from left to right, from down to up
regionDwellTimes={}; % tool dwell poses, from left to right, from down to up
instWorkpieceMR={}; % MR profile after each local polishing
instWorkpieceMeanMR={};
workpieceMR_0=zeros(length(workpiecePt),1);
workpieceMR_0(ptIndices(workpiecePt(ptIndices,1)>=0))=60; % define initial MR 30 10^-3 mm
workpieceMR=workpieceMR_0;
% workpieceMeanMR=exp(-mutualDists.^2/(2*(0.5*interestRadius)^2))*single(workpieceMR)/((0.5*interestRadius/gridSize)^2*2*pi);
workpieceMeanMR=localIDs*workpieceMR./sum(localIDs,2);
maxMeanMR=max(workpieceMeanMR);
initialMaxMeanMR=max(workpieceMeanMR);
% pcshow(workpiecePtCloudUp.Location,workpieceMR); 
% pcshow(workpiecePtCloudUp.Location,workpieceMeanMR);
%% search and polish
global stopMeanMR; stopMeanMR=initialMaxMeanMR/50;
global halfPathWidth; halfPathWidth=1*gridSize; %0.75*gridSize;
global halfPatternWidth; halfPatternWidth=gridSize; %0.5*Disc_R;
global minPathLength; minPathLength=20;
global holeLength; holeLength=20;
while maxMeanMR(end)>stopMeanMR && subregionCount<=50 % the end condition
    [centerID,neighborRegion]=findInterestRegion(toolPos,toolNor,workpieceMeanMR);
    [pathLocations,toolPoses,toolDwellTimes,workpieceMR]=localPathPlanning(centerID,polishForce,workpieceMR);
    subregionCount=subregionCount+1
    centerIDs(subregionCount)=centerID;
    localPath_x{subregionCount}=pathLocations;
    sampleToolPoses{subregionCount}=toolPoses;
    regionDwellTimes{subregionCount}=toolDwellTimes;   
    toolPos=workpiecePt(centerID,:);
    toolNor=workpieceNormals(centerID,:);                                                                             
    instWorkpieceMR{subregionCount}=workpieceMR;
%     workpieceMeanMR=exp(-mutualDists.^2/(2*(0.5*interestRadius)^2))*single(workpieceMR)/((0.5*interestRadius/gridSize)^2*2*pi);
    workpieceMeanMR=localIDs*workpieceMR./sum(localIDs,2);
    instWorkpieceMeanMR{subregionCount}=workpieceMeanMR;
    maxMeanMR=[maxMeanMR,max(workpieceMeanMR)];
end
pcshow(workpiecePt,workpieceMR);
% pcshow(workpiecePt,workpieceMeanMR);
%%
hold off
mean_MR=[];std_MR=[];minMeanMR=[];
for i=1:subregionCount
    mean_MR(i)=mean(instWorkpieceMR{i});
    std_MR(i)=std(instWorkpieceMR{i});
    minMeanMR(i)=min(instWorkpieceMeanMR{i});
end
plot(mean_MR);hold on
plot(std_MR)
plot(minMeanMR)
plot(maxMeanMR);
%%
workpieceMR_process=moviein(subregionCount);
workpieceMR_process(1)=getframe;
for frame=1:subregionCount
    pcshow(workpiecePt,instWorkpieceMR{frame});
    workpieceMR_process(1+frame)=getframe;
end
movie(workpieceMR_process,1,1)
%%
workpieceMeanMR_process=moviein(subregionCount);
workpieceMeanMR_process(1)=getframe;
for frame=1:subregionCount
    pcshow(workpiecePt,instWorkpieceMeanMR{frame});
    workpieceMeanMR_process(1+frame)=getframe;
end
movie(workpieceMeanMR_process,1,1)
%%
regionTime=zeros(subregionCount,1);
T=0;
for i=1:subregionCount
    for j=1:length(sampleToolPoses{i})
        for k=1:length(sampleToolPoses{i}{j})
            T=T+regionDwellTimes{i}{j}(k);
        end
    end
    regionTime(i)=T;
end
plot(regionTime,maxMeanMR(2:end));hold on
%%
pcshow(workpiecePt,workpieceMR);hold on;
for i=1:10
    for j=1:length(sampleToolPoses{i})
        c_pos=zeros(length(sampleToolPoses{i}{j}),3);
        for k=1:length(sampleToolPoses{i}{j})
            c_pos(k,:)=(sampleToolPoses{i}{j}(1:3,4,k)+pagemtimes(sampleToolPoses{i}{j}(1:3,1:3,k),[0 Disc_R 0]'))';
        end
        scatter3(c_pos(:,1),c_pos(:,2),c_pos(:,3),'r','.');hold on;
        scatter3(workpiecePt(centerIDs(i),1),workpiecePt(centerIDs(i),2),workpiecePt(centerIDs(i),3),'r','o');
    end
end