function nurbs = nrbs6dim(coefs,knots)
%
% Function Name:

nurbs.form   = 'B-NURBS';
nurbs.dim    = 6 ;
np = size(coefs);
% dim = np(1);
n=np(2);


% constructing a curve
nurbs.number = np(2);
nurbs.coefs = coefs;

if nargin < 2
    nurbs.order = 6;
    knots =zeros(1,n+6);
    for i=1:n
        
        knots(i+6)=knots(i+5)+1/(n+1)+0.01*rand(1);
        
    end
    knots(end-5:end)=1;
    nurbs.knots = knots;
else
    nurbs.order = length(knots)-n;
    nurbs.knots = knots;
end


end




