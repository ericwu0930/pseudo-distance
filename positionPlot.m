xbest = [98.0001  295.7219   40.7378    4.1700
  113.4295  275.8560   93.9307   40.5942
   84.3690  250.2763  230.8831  -42.7119
  188.1356  257.3483  248.7750   45.9894
  175.5428  219.8820  241.1875  282.1613
  121.1634  248.0977  290.6305  327.9905
  311.6967  270.1646  101.4491  295.0505
  269.9989  217.0669  105.1791  252.9520
  209.8452  164.5494   62.2050   52.2007
  236.0471  231.7990  -62.0164   16.2527 ];
for i = 1:size(xbest,1)
    resultp = fk(xbest(i,:));
    for j = 1:size(resultp,1)-1
        plot(resultp(j:j+1,1),resultp(j:j+1,2),'r-');
    end
end

function [x] = fk(theta)
% theta  1xdc configuration space of manipulator deg
% a0     position of manipulator's base
% l      the length of manipulator
% x      N x 2 or 3
a0 = [7 7.5];
l = 2;
dc = length(theta);
theta = theta/180*pi;
% 2-d environment
x = zeros(dc+1,2);
x(1,:) = a0(:)';
for i = 2:5
    x(i,:) = x(i-1,:)+[l*cos(theta(i-1)) l*sin(theta(i-1))];
end
end