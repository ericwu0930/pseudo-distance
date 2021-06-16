function dnurbs = my_nrbderiv(nurbs) 
%MY_NRBDERIV �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

  [dcoefs,dknots] = bspderiv(nurbs.order - 1,nurbs.coefs,nurbs.knots); 
  dnurbs = nrbs6dim(dcoefs, dknots); 
 
end

