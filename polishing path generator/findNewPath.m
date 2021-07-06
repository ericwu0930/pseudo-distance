function [newPath_x,dwellTime,toolPoses,deltaMR]=findNewPath(centerID,polishIDs,feedDirection,path_x,polishForce,lastDeltaMR,thisWorkpieceMR)
global workpiecePt
global workpieceNormals
global Disc_R
global gridSize
normal=workpieceNormals(centerID,:);
y_axis=feedDirection;
x_axis=cross(y_axis,normal);
Rot=[x_axis' y_axis' normal'];
center=workpiecePt(centerID,:);
projWorkpiecePt=(Rot\(workpiecePt-center)')';
projLocalPt=projWorkpiecePt(polishIDs,:);
last_MR=thisWorkpieceMR-lastDeltaMR;
regionRight=max(projLocalPt(:,1));
regionLeft=min(projLocalPt(:,1));
% golden section
alpha=0.618; 
minPathInterval=0.25*Disc_R;
if path_x>=0 % on the right side
    gold_a=min(regionRight,path_x+minPathInterval); % search space from 0.25R->R
    gold_b=regionRight;
else % on the left side
    gold_a=regionLeft;
    gold_b=max(regionLeft,path_x-minPathInterval);
end
weightNegativeMR=1; % if negative MR is not avoided, assuming not polished by adjacent paths
gold_lam=gold_a+(1-alpha)*(gold_b-gold_a);
gold_mu=gold_a+alpha*(gold_b-gold_a); 
[~,~,deltaMR_1]=findDwellTime(centerID,polishIDs,feedDirection,gold_lam,polishForce,last_MR);
workpieceMR_1=thisWorkpieceMR+deltaMR_1;
[region_ids_1,~]=find((projLocalPt(:,1)-gold_lam).*(projLocalPt(:,1)-path_x)<=0); % minimize bias in this area
region_IDs_1=polishIDs(region_ids_1);
meanMRbias_1=sum(workpieceMR_1(region_IDs_1).^2.*(1+(weightNegativeMR^2-1)*(workpieceMR_1(region_IDs_1)<0)))/length(region_IDs_1); % f1
f1=meanMRbias_1;
[~,~,deltaMR_2]=findDwellTime(centerID,polishIDs,feedDirection,gold_mu,polishForce,last_MR);
workpieceMR_2=thisWorkpieceMR+deltaMR_2;
[region_ids_2,~]=find((projLocalPt(:,1)-gold_mu).*(projLocalPt(:,1)-path_x)<=0);
region_IDs_2=polishIDs(region_ids_2);
meanMRbias_2=sum(workpieceMR_2(region_IDs_2).^2.*(1+(weightNegativeMR^2-1)*(workpieceMR_2(region_IDs_2)<0)))/length(region_IDs_2); % f2
f2=meanMRbias_2;
goldTolerance=2*gridSize; % threshold
while(gold_b-gold_a>=goldTolerance) 
    if f1>=f2
        gold_a=gold_lam;
        gold_lam=gold_mu;
        gold_mu=gold_a+alpha*(gold_b-gold_a);
        f1=f2;
        [~,~,deltaMR_2]=findDwellTime(centerID,polishIDs,feedDirection,gold_mu,polishForce,last_MR);
        workpieceMR_2=thisWorkpieceMR+deltaMR_2;
        [region_ids_2,~]=find((projLocalPt(:,1)-gold_mu).*(projLocalPt(:,1)-path_x)<=0);
        region_IDs_2=polishIDs(region_ids_2);
        meanMRbias_2=sum(workpieceMR_2(region_IDs_2).^2.*(1+(weightNegativeMR^2-1)*(workpieceMR_2(region_IDs_2)<0)))/length(region_IDs_2); % f2 
        f2=meanMRbias_2;
    else
        gold_b=gold_mu;
        gold_mu=gold_lam;
        gold_lam=gold_a+(1-alpha)*(gold_b-gold_a);
        f2=f1;
        [~,~,deltaMR_1]=findDwellTime(centerID,polishIDs,feedDirection,gold_lam,polishForce,last_MR);
        workpieceMR_1=thisWorkpieceMR+deltaMR_1;
        [region_ids_1,~]=find((projLocalPt(:,1)-gold_lam).*(projLocalPt(:,1)-path_x)<=0); % minimize bias between adjacent paths
        region_IDs_1=polishIDs(region_ids_1);
        meanMRbias_1=sum(workpieceMR_1(region_IDs_1).^2.*(1+(weightNegativeMR^2-1)*(workpieceMR_1(region_IDs_1)<0)))/length(region_IDs_1); % f1
        f1=meanMRbias_1;
    end
end
newPath_x=(gold_a+gold_b)/2;
edgeMargin=gridSize;
if newPath_x>regionRight-edgeMargin || newPath_x<regionLeft+edgeMargin % the path_x cannot be reach limit, save time
    dwellTime=[];
    toolPoses=[];
    deltaMR=zeros(length(workpiecePt),1);
else  % recalculate dwell time
    [dwellTime,toolPoses,deltaMR]=findDwellTime(centerID,polishIDs,feedDirection,newPath_x,polishForce,thisWorkpieceMR); 
end
