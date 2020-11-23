clear all;
close all;
clc;
tic
%% DE Algorithm to solve unconstrained optimization
global rk;
rk = 0.5; c = 1.5;
times = 0;
%% function to be minimized
D=3; % number of variables
%% initialization of de parameters
N=20;     % same generation population size
itmax=30; % number of iterations
F=0.8;CR=0.5; % mutation factor and crossover ratio
% problem bounds
a=-1.9; b=1.9; % bounds on variable x1
d=(b-a);
basemat=repmat(int16(linspace(1,N,N)),N,1); % used later
basej=repmat(int16(linspace(1,D,D)),N,1); % used later
while 1
    times = times+1;
    disp(['Loop times',times,'. ','rk=',rk]);
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
        fu=objf(u);
        idx=fu<fx;
        fx(idx)=fu(idx);
        x(idx,1:D)=u(idx,1:D);
        % find best
        [fxbest,ixbest]=min(fx);
        xbest=x(ixbest,1:D);
    end
    hold on;
    plot(times,fxbest,'ro');
    if penalty(xbest)<1e-3
        break;
    end
    rk = c*rk;
end
[xbest,fxbest]
toc

function fval = objf(x)
fval = x(:,1).^3-6*x(:,1).^2+11*x(:,1)+x(:,3);
fval = fval+penalty(x);
end

function res = penalty(x)
global rk;
res = zeros(size(x,1),1);
p = zeros(size(x,1),6);
tmp = zeros(size(x,1),1);
p(:,1) = x(:,1).^2+x(:,2).^2-x(:,3).^2;
p(:,2) = 4-x(:,1).^2-x(:,2).^2-x(:,3).^2;
p(:,3) = x(:,3)-5;
p(:,4) = -x(:,1);
p(:,5) = -x(:,2);
p(:,6) = -x(:,3);
for i = 1:6
    idx = tmp<p(:,i);
    res = res+idx.*p(:,i).^2*rk;
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