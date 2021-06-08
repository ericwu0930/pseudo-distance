function p = my_nrbeval(nurbs,tt) 
%  
% Function Name: 
%  
%   nrbeval - Evaluate a NURBS at parameteric points 
%  
% Calling Sequence: 
%  
%   [p,w] = nrbeval(crv,ut) 
%   [p,w] = nrbeval(srf,{ut,vt}) 
%  
% Parameters: 
%  
%   crv		: NURBS curve, see nrbmak. 
%  
%   srf		: NURBS surface, see nrbmak. 
%   
%   ut		: Parametric evaluation points along U direction. 
% 
%   vt		: Parametric evaluation points along V direction. 
%  
%   p		: Evaluated points on the NURBS curve or surface as cartesian 
% 		coordinates (x,y,z). If w is included on the lhs argument list 
% 		the points are returned as homogeneous coordinates (wx,wy,wz). 
%  
%   w		: Weights of the homogeneous coordinates of the evaluated 
% 		points. Note inclusion of this argument changes the type  
% 		of coordinates returned in p (see above). 
%  
% Description: 
%  
%   Evaluation of NURBS curves or surfaces at parametric points along the  
%   U and V directions. Either homogeneous coordinates are returned if the  
%   weights are requested in the lhs arguments, or as cartesian coordinates. 
%   This function utilises the 'C' interface bspeval. 
%  
% Examples: 
%  
%   Evaluate the NURBS circle at twenty points from 0.0 to 1.0 
%  
%   nrb = nrbcirc; 
%   ut = linspace(0.0,1.0,20); 
%   p = nrbeval(nrb,ut); 
%  
% See: 
%   
%     bspeval 
% 
 
%  D.M. Spink 
%  Copyright (c) 2000. 
 
 
  % NURBS structure represents a curve 
  %  tt represent a vector of parametric points in the u direction 
 
  val = bspeval(nurbs.order-1,nurbs.coefs,nurbs.knots,tt); 
  p=val;
 
 
 
end 
