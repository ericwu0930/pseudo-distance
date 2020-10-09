vertices=[0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1];
faces=[1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8 ];
for i = 1 : 6
    h = patch(vertices(faces(i,:),1),vertices(faces(i,:),2),vertices(faces(i,:),3),'g');
    set(h,'facealpha',0.2);
end
axis equal
for i = 1 : 8
    text(vertices(i,1),vertices(i,2),vertices(i,3),num2str(i));
end
axis off
view(3);
