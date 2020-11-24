clear all;
close all;
clc;
tic
%% DE Algorithm to solve unconstrained optimization
global rk;
rk = 0.5; c = 1.5;
global itmax;
global it;
global e0;
e0 = 15;
times = 0;
%% function to be minimized
D=3; % number of variables
%% initialization of de parameters
N=20;     % same generation population size
itmax=300; % number of iterations
F=0.8;CR=0.5; % mutation factor and crossover ratio
% problem bounds
a=2; b=5; % bounds on variable x1
d=(b-a);
basemat=repmat(int16(linspace(1,N,N)),N,1); % used later
basej=repmat(int16(linspace(1,D,D)),N,1); % used later
% random initialization of postions
x=a+d.*rand(N,D);
% evaluate objective for all particles
fx=objf(x);
% find bset
[fxbest,ixbest] = min(fx);
xbest = x(ixbest,1:D);
%% iterate
for it=1:itmax
    permat=bsxfun(@(x,y) x(randperm(y(1))),basemat',N(ones(N,1)))';
    % generate donors by mutation
    v(1:N,1:D)=repmat(xbest,N,1)+F*(x(permat(1:N,1),1:D)-x(permat(1:N,2),1:D));
    % perform recobination
    r=repmat(randi([1 D],N,1),1,D);
    muv=((rand(N,D)<CR)+(basej==r))~=0;
    mux=1-muv;
    u(1:N,1:D)=x(1:N,1:D).*mux(1:N,1:D)+v(1:N,1:D).*muv(1:N,1:D);
    % greedy selection
    idx = prior(u,x);
    idx = logical(idx);
    x(idx,:) = u(idx,:);
    fx=objf(x);
    % find best
    [fxbest,ixbest]=min(fx);
    xbest=x(ixbest,1:D);
    hold on;
    plot(it,fxbest,'ro');
end
[xbest,fxbest]
toc

function fval = objf(x)
fval = x(:,1).^3-6*x(:,1).^2+11*x(:,1)+x(:,3);
end

function res = constraint(x)
res = zeros(size(x,1),1);
p = zeros(size(x,1),6);
tmp = zeros(size(x,1),1);
p(:,1) = x(:,1).^2+x(:,2).^2-x(:,3).^2;
p(:,2) = 4-x(:,1).^2-x(:,2).^2-x(:,3).^2;
p(:,3) = x(:,3)-5;
p(:,4) = -x(:,1);
p(:,5) = -x(:,2);
p(:,6) = -x(:,3);
% for i = 1:6
%     idx = tmp<p(:,i);
%     res = res+idx.*p(:,i);
% end
idx = tmp<p;
res = sum(idx.*p,2);
end

function epsilon = relaxation(times)
global itmax;
global e0;
if times>=itmax
    epsilon = 0;
else
    epsilon = e0*(1-times/itmax)^3;
end
end

function [pri] = prior(x1,x2)
global it;
c1=constraint(x1);
c2=constraint(x2);
fx1 = objf(x1);
fx2 = objf(x2);
pri = zeros(size(x1,1),1);
idx1 = c1==0;
idx2 = c2==0;
for i = 1:size(idx1,1)
    if idx1(i) && idx2(i) == 1 %都在可行域内
        pri(i) = fx1(i)<fx2(i);
    elseif idx1(i) || idx2(i) == 0
        pri(i) = c1(i)<c2(i);
    else
        if idx1(i) == 0
            if c1(i) < relaxation(it)
                pri(i) = fx1(i) < fx2(i);
            else
                pri(i) = 0;
            end
        else
            if c2(i) < relaxation(it)
                pri(i) = fx1(i)<fx2(i);
            else
                pri(i) = 1;
            end
        end
    end
end
end

% function fval = objf(x)
% fval = 1/3*(x(:,1)+1).^3+x(:,2);
% fval = fval+penalty(x);
% end
%
% function res = penalty(x)
% global rk;
% res = zeros(size(x,1),1);
% p = zeros(size(x,1),2);
% tmp = zeros(size(x,1),1);
% p(:,1) = -x(:,1)+1;
% p(:,2) =-x(:,2);
% for i = 1:2
%     idx = tmp<p(:,i);
%     res = res+idx.*p(:,i).^2*rk;
% end
% end