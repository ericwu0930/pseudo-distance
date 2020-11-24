function [rho,fval,DT] = QDistance(A,B,Q)
% calculate the dQ(A,B), where A and B are polyhedra exprerssed by vertices
% A or B: n*m matrix
% [VecDim,QVDim] = size(Q);
[DT,bdidx] = MinkowskiSum(A',-B');
% D = DT.Points(DT.convexHull,:);
% AlphaDim = length(DT.convexHull);
% D = DT';
% AlphaDim = size(D,2);
HaveIntersection = 1;
BDMeet = 0;
VolTmpOld = det(DT(bdidx(1,:),:));
for i=2:size(bdidx)
    VolTmp = det(DT(bdidx(i,:),:));
    if VolTmp*VolTmpOld>0
        VolTmpOld = VolTmp;
    elseif VolTmp*VolTmpOld<0
        HaveIntersection = 0;
        break;
    else
        BDMeet = 1;
        break;
    end
end

if ~BDMeet
    [ra,ca]=size(A);
    [rb,~]=size(B);
    [rq,~]=size(Q);
    if ~HaveIntersection % 稍作改造 如果没有交集即符合约束则返回0
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
        for i = 1:rq
            row1 = [A',B',-Q(i,:)'];
            row2 = [ones(1,ra),zeros(1,rb+1)];
            row3 = [zeros(1,ra),ones(1,rb),0];
            Aeq = [row1;row2;row3];
            beq = [zeros(ca,1);1;1];
            f = [zeros(1,ra+rb),-1];
            lb = zeros(size(f));
            [x,fval] = linprog(f,[],[],Aeq,beq,lb,[]);
            if(fval>maxFval)
                maxX = x;
                maxFval = fval;
            end
        end
        rho=maxX;
        fval=maxFval;
    end
else
    rho = 0; fval = 0;
end
end




