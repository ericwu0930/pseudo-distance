% region lift, path lift, hole lift
global Disc_R
sampleDistance=1;
globalPos=[0 0 200];
globalQuater=UnitQuaternion(eye(3));
globalV=50; % move in the air
globalFlag=5;
for i=1:15 % mean<=2
    localPos=[];
    localQuater=[];
    localV=[];
    localFlag=[];
    for j=1:length(regionDwellTimes{i})
        if isempty(sampleToolPoses{i}{j}) % if this path is empty, ignore it
            continue
        end
        if isempty(localPos)
            nextDistance1=norm(globalPos(end,:)-sampleToolPoses{i}{j}(1:3,4,2)');
            nextDistance2=norm(globalPos(end,:)-sampleToolPoses{i}{j}(1:3,4,end-1)');
        else
            nextDistance1=norm(localPos(end,:)-sampleToolPoses{i}{j}(1:3,4,2)');
            nextDistance2=norm(localPos(end,:)-sampleToolPoses{i}{j}(1:3,4,end-1)');
        end
        localPos=cat(1,localPos,sampleToolPoses{i}{j}(1:3,4,end-1)');
        localQuater=cat(1,localQuater,UnitQuaternion(sampleToolPoses{i}{j}(1:3,1:3,end-1)));
        localV=cat(1,localV,3);
        localFlag=cat(1,localFlag,2);
        for k=length(sampleToolPoses{i}{j})-2:-1:3
            if norm(sampleToolPoses{i}{j}(1:3,4,k-1)-sampleToolPoses{i}{j}(1:3,4,k))>20
                localPos=cat(1,localPos,sampleToolPoses{i}{j}(1:3,4,k)');
                localQuater=cat(1,localQuater,UnitQuaternion(sampleToolPoses{i}{j}(1:3,1:3,k)));
                localV=cat(1,localV,sampleDistance./regionDwellTimes{i}{j}(k));
                localFlag=cat(1,localFlag,-2);
            elseif norm(sampleToolPoses{i}{j}(1:3,4,k+1)-sampleToolPoses{i}{j}(1:3,4,k))>20
                localPos=cat(1,localPos,sampleToolPoses{i}{j}(1:3,4,k)');
                localQuater=cat(1,localQuater,UnitQuaternion(sampleToolPoses{i}{j}(1:3,1:3,k)));
                localV=cat(1,localV,3);
                localFlag=cat(1,localFlag,2);
            else
                localPos=cat(1,localPos,sampleToolPoses{i}{j}(1:3,4,k)');
                localQuater=cat(1,localQuater,UnitQuaternion(sampleToolPoses{i}{j}(1:3,1:3,k)));
                localV=cat(1,localV,sampleDistance./regionDwellTimes{i}{j}(k));
                localFlag=cat(1,localFlag,0);
            end
        end
        localPos=cat(1,localPos,sampleToolPoses{i}{j}(1:3,4,2)');
        localQuater=cat(1,localQuater,UnitQuaternion(sampleToolPoses{i}{j}(1:3,1:3,2)));
        localV=cat(1,localV,sampleDistance./regionDwellTimes{i}{j}(2));
        localFlag=cat(1,localFlag,-2);
    end
    localFlag(1)=3;
    localFlag(end)=-3;
    globalPos=cat(1,globalPos,localPos);
    globalQuater=cat(1,globalQuater,localQuater);
    globalV=cat(1,globalV,localV);
    globalFlag=cat(1,globalFlag,localFlag);
end
globalPos=cat(1,globalPos,[0 0 200]);    % the last point
globalQuater=cat(1,globalQuater,UnitQuaternion(eye(3)));
globalV=cat(1,globalV,50);
globalFlag=cat(1,globalFlag,5);
globalPos=double(globalPos);
globalQuater=double(globalQuater);
globalV=double(globalV);
%% plot paths
generatedPaths=moviein(length(globalPos));
pcshow(workpiecePt,workpieceMR);
hold on;
for i=1:length(globalPos)
    Rot=UnitQuaternion.q2r(globalQuater(i,:));
    cPos=globalPos(i,:)'+Rot*[0 Disc_R*1 0]';
    scatter3(cPos(1),cPos(2),cPos(3),'r','.');
    %     quiver3(cPos(1),cPos(2),cPos(3),R(1,3),R(2,3),R(3,3),20,'LineWidth',1,'Color','r');
    generatedPaths(i)=getframe;
end
movie(generatedPaths,1,10);