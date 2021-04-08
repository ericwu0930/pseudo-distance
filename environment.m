% test evironment preparation for Oriolo's paper
global l a0 n m;
l = 2;
a0 = [0 0];
n = 4; % cnt of joints is 4
m = 3; % x,y,theta
inv_fkine = @ikine;
for_kine = @fkine;

function [q_b,succ] = ikine(p,qr,q_bias)

end

function fkine()
end

