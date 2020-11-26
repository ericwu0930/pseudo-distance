%% 线性规划实现Q- distance
clear;
clc;
close all;
% G = [1,6;
%     5,6;
%     3,2];
G = [8.0775,9.1849;
   10.0447,9.5459];
% H = [3,3;
%     1,1;
%     5,1];
H = [9.0000,9.5000;10.0000,9.5000;10.0000,18.0000;9.0000,18.0000];
patch(G(:,1),G(:,2),'r')
hold on
axis on
axis equal
grid on
patch(H(:,1),H(:,2),'b')
meanG = mean(G);
meanH = mean(H);
text(meanG(1),meanG(2),'G');
text(meanH(1),meanH(2),'H');
Q = [0,-1;
    sqrt(3)/2,1/2;
    -sqrt(3)/2,1/2];
[rg,cg]=size(G);
[rh,ch]=size(H);
[rq,cq]=size(Q);
P = ones(rg*rh,2);
for i = 1:rg
    for j = 1:rh
        P((i-1)*rh+j,:) = G(i,:)-H(j,:);
    end
end
K = convhull(P(:,1),P(:,2));
meanP = mean(P);
patch(P(K,1),P(K,2),'w');
text(meanP(1),meanP(2),'G-H');
maxFval = -inf;
maxX = [];
for i = 1:rq
    row1 = [G' -H' -Q(i,:)'];
    row2 = [ones(1,rg),zeros(1,rh),0];
    row3 = [zeros(1,rg),ones(1,rh),0];
    Aeq = [row1;row2;row3];
    beq = [zeros(cg,1);1;1];
    f = [zeros(1,rg+rh),-1];
    lb = zeros(size(f));
    [x,fval] = linprog(f,[],[],Aeq,beq,lb,[]);
    if(fval>maxFval)
        maxX = x;
        maxFval = fval;
    end
end
maxFval;
newQ = -maxFval*Q;
newQ
newQ = [newQ;newQ(1,:)]


for i = 1:3
plot([newQ(i,1),newQ(i+1,1)],[newQ(i,2),newQ(i+1,2)],'r-');
end

% patch(Q(:,1),Q(:,2),'r');


function p=createSimplyPoly(p)
    cen=mean(p);
    ang=atan2(p(:,1)-cen(1),p(:,2)-cen(2)); %每个点到坐标中心极角

    p=[p,ang];
    p=sortrows(p,3);    %按极角排序

    p=p(:,1:2);
end