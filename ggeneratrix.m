function [j1g, j2g] = ggeneratrix(j1, j2, vertexes, R)
    %% Comment
    % 得到距离障碍平面最近的母线
    % j1,j2 连杆两端点
    % vertexes 障碍平面顶点,需要按照顺时针或者逆时针给出
    % R 连杆的半径
    %% Body
    [a, b, c, ~] = gplane(vertexes(1, :), vertexes(2, :), vertexes(3, :));
    D = gD(j1, j2, [a b c]);
    % 选择指向平面方向的D
    if D(:)' * (vertexes(1, :) - j1) < 0
        D = -D;
    end

    j1g = j1 + R * D;
    j2g = j2 + R * D;
end

function [D] = gD(j1, j2, n)
    %% Comments
    % 得到中心线向障碍平面投影方向的向量d
    %% Body
    j1 = j1(:);
    j2 = j2(:);
    n = n(:);
    j1j2 = j2 - j1;
    A = [j1j2(1) j1j2(2) j1j2(3);
        n(1) n(2) n(3);
        0 0 1];
    b = [0; 0; 1];
    nx = A \ b;
    A = [j1j2(1) j1j2(2) j1j2(3);
        nx(1) nx(2) nx(3);
        0 0 1];
    D = A \ b;
    D = D / norm(D);
end

function [case1] = dcase(j1, j2, vertexes)
    %% Comments
    % determine case
    % 给定j1,j2 判断向障碍平面的投影是否有部分在障碍平面内
    %% Body
    % 先将线段往障碍物平面投影，然后判断与障碍物平面的边界是否有交线
    [a, b, c, d] = gplane(vertexes(1, :), vertexes(2, :), vertexes(3, :));
    j1p = projToPlane(j1, a, b, c, d);
    j2p = projToPlane(j2, a, b, c, d);
    % 如果两个点都在障碍物平面内，则为论文中的case1
    if inBound(j1p, vertexes) && inBound(j2p, vertexes)
        case1 = true;
        return;
    end

    % 首尾相连 增加一行
    vertexes = [vertexes; vertexes(1, :)];

    for i = 1:size(vertexes, 1) - 1

        if lineIntersect(j1p, j2p, vertexes(i, :), vertexes(i + 1, :)) > 0
            case1 = true;
            return;
        end

    end

    case1 = false;
end
