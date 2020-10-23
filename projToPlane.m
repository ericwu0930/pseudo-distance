function [o] = projToPlane(o,a,b,c,d)
%% Comments
% 将一点投影到一平面上
% n表示该平面的法线
%% Body
f = @(x,o,a,b,c,d) [a b c]*(o(:)+x*[a;b;c])+d;
x=fsolve(@(x) f(x,o,a,b,c,d),0);
o = x*[a;b;c]+o(:);
end