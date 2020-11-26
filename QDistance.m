function [rho,fval,DT] = QDistance(A,B,Q)
% calculate the dQ(A,B), where A and B are polyhedra exprerssed by vertices
% A or B: n*m matrix
[VecDim,QVDim] = size(Q);
[DT,bdidx] = MinkowskiSum(A',-B');
% D = DT.Points(DT.convexHull,:);
% AlphaDim = length(DT.convexHull);
D = DT';
AlphaDim = size(D,2);
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
    if ~HaveIntersection % 稍作改造 如果没有交集即符合约束则返回0
%         f = [ones(1,QVDim), zeros(1,AlphaDim)];
%         lb = zeros(1,QVDim+AlphaDim);
%         beq = [zeros(1,VecDim) 1]';
%         Aeq = [Q -D; zeros(1,QVDim) ones(1,AlphaDim)];
%         [rho,fval] = linprog(f,[],[],Aeq,beq,lb,[]);
        rho=0;fval=0;
    else
        f = [-1 zeros(1,AlphaDim*QVDim)];
        lb = zeros(1,1+AlphaDim*QVDim);
        Aeq = [Q(:) -kron(eye(QVDim),D); zeros(QVDim,1) kron(eye(QVDim),ones(1,AlphaDim))];
        beq = [zeros(1,QVDim*VecDim) ones(1,QVDim)]';
        [rho,fval] = linprog(f,[],[],Aeq,beq,lb,[]);
    end
else
    rho = 0; fval = 0;
end
end