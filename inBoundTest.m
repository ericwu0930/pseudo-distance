%% Comments
% 测试inBound算法正确性 https://www.cnblogs.com/tiandsp/p/4019880.html
%% Body
clear;
clc;
close all;

polyn=20;
poly=rand(polyn,2);
poly=createSimplyPoly(poly);  %创建简单多边形

hold on;
plot(poly(:,1),poly(:,2),'b');
plot(poly([1,size(poly,1)],1),poly([1,size(poly,1)],2),'b');

pn=500;
p=rand(pn,2);
for i=1:pn
%     plot(p(i,1),p(i,2),'r.');
%     for j=2:polyn
%         x1=poly(j-1,1);         %多边形前后两个点
%         y1=poly(j-1,2);
%         x2=poly(j,1);
%         y2=poly(j,2);
%        
%         k=(y1-y2)/(x1-x2);      %多边形一条边直线
%         b=y1-k*x1;
%         x=p(i,1);               %过当前点直线和多边形交点
%         y=k*x+b;          
%                
%         if min([x1 x2])<=x && x<=max([x1 x2]) && ...        %点同时在射线和多边形边上
%            min([y1 y2])<=y && y<=max([y1 y2]) &&  y>=p(i,2)
%                flag=flag+1;
%         end
%     end
    flag = inBound(p(i,:),poly);
   
    if flag == false               %偶数则在外部
        plot(p(i,1),p(i,2),'r.');
    else                            %奇数则在内部
        plot(p(i,1),p(i,2),'g.');        
    end
end

%% 随机多边形自动生成
function p=createSimplyPoly(p)
    cen=mean(p);
    ang=atan2(p(:,1)-cen(1),p(:,2)-cen(2)); %每个点到坐标中心极角

    p=[p,ang];
    p=sortrows(p,3);    %按极角排序

    p=p(:,1:2);
end