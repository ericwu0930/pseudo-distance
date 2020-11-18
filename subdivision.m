function [vertex, face] = subdivision(vertex, face)
% 球面细分算法
face_num = size(face, 1);
vertex_num = size(vertex, 1);
new_vertexs = zeros(face_num*3, 3);
new_faces = zeros(face_num*3, 3);
for i = 1 : face_num
    % 取中点
    new_vertexs(3*(i-1)+1, :)=(vertex(face(i, 1), :)+vertex(face(i, 2), :))/2;
    new_vertexs(3*(i-1)+2, :)=(vertex(face(i, 2), :)+vertex(face(i, 3), :))/2;
    new_vertexs(3*(i-1)+3, :)=(vertex(face(i, 3), :)+vertex(face(i, 1), :))/2;
    % 正规化
    new_vertexs(3*(i-1)+1, :)= new_vertexs(3*(i-1)+1, :)/norm(new_vertexs(3*(i-1)+1, :));
    new_vertexs(3*(i-1)+2, :)= new_vertexs(3*(i-1)+2, :)/norm(new_vertexs(3*(i-1)+2, :));
    new_vertexs(3*(i-1)+3, :)= new_vertexs(3*(i-1)+3, :)/norm(new_vertexs(3*(i-1)+3, :));
    
    new_faces(3*(i-1)+1, :) = [face(i, 1), vertex_num+3*(i-1)+1, vertex_num+3*(i-1)+3];
    new_faces(3*(i-1)+2, :) = [face(i, 2), vertex_num+3*(i-1)+2, vertex_num+3*(i-1)+1];
    new_faces(3*(i-1)+3, :) = [face(i, 3), vertex_num+3*(i-1)+3, vertex_num+3*(i-1)+2];
    face(i, :) = [vertex_num+3*(i-1)+1, vertex_num+3*(i-1)+2, vertex_num+3*(i-1)+3];
end
face = [face; new_faces];
vertex = [vertex; new_vertexs];