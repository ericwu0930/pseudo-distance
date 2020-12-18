function [time]=testToc()
tic;
for i= 1:1000000
    a=sqrt(3);
end
time = toc;
end