clear all;
path=[0.7854  0   -1.0472
    0.9027    0.0984   -0.9185
    1.0396    0.1275   -0.6703
    1.3124    0.1523   -0.4858
    1.5999    0.1767   -0.3755
    1.9053    0.2324   -0.2131
    2.0465    0.3718   -0.0900
    2.1870    0.5883    0.0014
    2.4693    0.6796    0.1784
    2.5488    0.8033    0.3139
    2.7323    1.0102    0.5987
    2.9394    1.1610    0.7724
    3.1994    1.3086    0.8979
    3.4286    1.5515    1.1008
    3.4944    1.6086    1.2808
    3.4945    1.8191    1.5924
    3.4931    1.9808    1.7101
    3.4907    2.2689    1.9199];
[r,~]=size(path);
i = 1;
epsilon = norm(path(1,:)-path(2,:))/3; % 转角误差
func = @(u,p1,d,t1,t2) p1-d*(1-u).^3.*t1+d*u.^3.*t2;
pathNew = [];
while i<r
    pathNew = [pathNew;path(i,:)];
    if i+2 > r
        pathNew = [pathNew;path(end,:)];
        break;
    end
    p0=path(i,:);
    p1=path(i+1,:);
    p2=path(i+2,:);
    t1=(p1-p0)/norm(p1-p0);
    t2=(p2-p1)/norm(p2-p1);
    theta = acos(dot(t1,t2));
    d = 4*epsilon*(1/sin(theta/2));
    minL = min(norm(p1-p0),norm(p2-p1));
    if d>minL
        d = minL;
    end
    u = linspace(0,1,50);
    u = u';
    pathNew = [pathNew;func(u,p1,d,t1,t2)];
    i=i+2;
end



