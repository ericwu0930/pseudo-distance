clear all;
close all;
clc;
tic
%% DE Algorithm to solve unconstrained optimization
%% function to be minimized
D=2; % number of variables
% objf=inline('4*x1^2-2.1*x1^4+(x1^6)/3+x1*x2-4*x2^2+4*x2^4','x1','x2');
% objf=vectorize(objf);
objf=@(x1,x2) 4*x1.^2-2.1*x1.^4+(x1.^6)/3+x1.*x2-4*x2.^2+4*x2.^4;
%% initialization of de parameters
N=20;     % same generation population size
itmax=30; % number of iterations
F=0.8;CR=0.5; % mutation factor and crossover ratio
% problem bounds
a(1:N,1)=-1.9; b(1:N,1)=1.9; % bounds on variable x1
a(1:N,2)=-1.1; b(1:N,2)=1.1; % bounds on variable x2
d=(b-a);
basemat=repmat(int16(linspace(1,N,N)),N,1); % used later
basej=repmat(int16(linspace(1,D,D)),N,1); % used later
% random initialization of postions
x=a+d.*rand(N,D);
% evaluate objective for all particles
fx=objf(x(:,1),x(:,2));
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
    fu=objf(u(:,1),u(:,2));
    idx=fu<fx;
    fx(idx)=fu(idx);
    x(idx,1:D)=u(idx,1:D);
    % find best
    [fxbest,ixbest]=min(fx);
    xbest=x(ixbest,1:D);
end
[xbest,fxbest]
toc