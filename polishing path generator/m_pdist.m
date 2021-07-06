function [D] = m_pdist(Workpiece_pt,Workpiece_normals)
global weightOrient; 
D=squareform(pdist(Workpiece_pt))+weightOrient*acos(min(max(Workpiece_normals*Workpiece_normals',-1),1));
end

