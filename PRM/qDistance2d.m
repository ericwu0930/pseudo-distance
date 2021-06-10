function [rho,fval,qe] = qDistance2d(A,B,Q)
qe = [];
shape1.XData = A(:,1);
shape1.YData = A(:,2);
shape2.XData = B(:,1);
shape2.YData = B(:,2);
isCol = gjk2d(shape1,shape2,6);
[ra,ca]=size(A);
[rb,~]=size(B);
[rq,~]=size(Q);
if ~isCol % 稍作改造 如果没有交集即符合约束则返回0
    row1 = [A',-B', -Q'];
    row2 = [ones(1,ra),zeros(1,length(row1)-ra)];
    row3 = [zeros(1,ra),ones(1,rb),zeros(1,length(row1)-ra-rb)];
    Aeq = [row1;
        row2;
        row3];
    beq = [zeros(ca,1);1;1];
    f = [zeros(1,ra+rb),ones(1,rq)];
    lb = zeros(size(f));
    [rho,fval] = linprog(f,[],[],Aeq,beq,lb,[]);
else
    maxFval = -inf;
    maxX = [];
    for i = 1:rq
        row1 = [A' -B' -Q(i,:)'];
        row2 = [ones(1,ra),zeros(1,rb),0];
        row3 = [zeros(1,ra),ones(1,rb),0];
        Aeq = [row1;row2;row3];
        beq = [zeros(ca,1);1;1];
        f = [zeros(1,ra+rb),-1];
        lb = zeros(size(f));
        [x,fval] = linprog(f,[],[],Aeq,beq,lb,[]);
        if(fval>maxFval)
            maxX = x;
            maxFval = fval;
            qe = i;
        end
    end
    rho=maxX;
    fval=maxFval;
end
end




