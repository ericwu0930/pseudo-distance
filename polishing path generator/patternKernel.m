function [y]=patternKernel(x)
y=(1-abs(x).^0).^0;
% y=(1-abs(x).^1).^1; 
% y=(1-abs(x).^2).^2; 