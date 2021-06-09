classdef Node
    properties
        type;
        coordinate;
        adjacent_coordinate;
        adjacent_index;
        cost;
    end
    
    methods
        function N = Node(Coordinate,Type)
            N.type = Type;
            N.coordinate = Coordinate;
        end
        
        function N = addAdjacentNode(N,adjacent_node,cost,index)
            N.adjacent_coordinate = [N.adjacent_coordinate;adjacent_node];
            N.adjacent_index = [N.adjacent_index;index];
            N.cost = [N.cost;cost];
        end
    end
end
