% 效果对比
pathLen1 = 0;
for i = 1:20
    rrtForZhu;
    pathLen1 = pathLen1 + pathLength;
end

pathLen2 = 0;
for i = 1:20
    rrtForZhuM;
    pathLen2 = pathLen2 + pathLength;
end