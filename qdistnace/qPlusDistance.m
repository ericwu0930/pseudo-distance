%% 线性规划实现Q+ distance
clear;
clc;
figure(2);
G = [-2,1;
    -3,-1;
    -1,-1];
H = [3,5;
    5,4;
    4,2];
patch(G(:,1),G(:,2),'y')
meanG = mean(G);
text(meanG(1),meanG(2),'G');
hold on 
grid on 
axis equal
patch(H(:,1),H(:,2),'b')
meanH = mean(H);
text(meanH(1),meanH(2),'H');
Q = [0,-1;
    sqrt(3)/2,1/2;
    -sqrt(3)/2,1/2];
meanQ = mean(Q);
patch(Q(:,1),Q(:,2),'r');
text(meanQ(1),meanQ(2),'Q');
P = ones(9,2);
for i = 1:3
    for j = 1:3
        P((i-1)*3+j,:) = G(i,:)-H(j,:);
    end
end
P = createSimplyPoly(P);
meanP = mean(P);
patch(P(:,1),P(:,2),'w');
text(meanP(1),meanP(2),'G-H');
row1 = [G(1,:)',G(2,:)',G(3,:)',-H(1,:)',-H(2,:)',-H(3,:)',-Q(1,:)',-Q(2,:)',-Q(3,:)'];
row2 = [ones(1,3),zeros(1,length(row1)-3)];
row3 = [zeros(1,3),ones(1,3),zeros(1,length(row1)-6)];
Aeq = [row1;
    row2;
    row3];
beq = [zeros(2,1);1;1];
f = [zeros(1,6),ones(1,3)];
lb = zeros(size(f));
[x,fval] = linprog(f,[],[],Aeq,beq,lb,[]);
fval
newQ = fval*Q;
newQ = [newQ;newQ(1,:)]


for i = 1:3
plot([newQ(i,1),newQ(i+1,1)],[newQ(i,2),newQ(i+1,2)],'r-');
end
meanQ = mean(newQ);
text(meanQ(1),meanQ(2),'newQ');


function p=createSimplyPoly(p)
    cen=mean(p);
    ang=atan2(p(:,1)-cen(1),p(:,2)-cen(2)); %每个点到坐标中心极角

    p=[p,ang];
    p=sortrows(p,3);    %按极角排序

    p=p(:,1:2);
end