%% define end effector trajactory
theta = pi/2;
f = @(x) -0.0839*x.^2+5.5;
x = [-2*sqrt(2),2*sqrt(2),200];
y = f(x);
