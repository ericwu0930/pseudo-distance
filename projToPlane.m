function [o] = projToPlane(o,a,b,c,d)
%% Comments
% 将一点投影到一平面上
% n表示该平面的法线
%% Body
x=fsolve(@(x) myfunc(x,o,a,b,c,d),1);
o = x*[a;b;c]+o;
end

function f=myfunc(x,o,a,b,c,d)
f = [a b c]*(o(:)+x*[a;b;c])+d;
end