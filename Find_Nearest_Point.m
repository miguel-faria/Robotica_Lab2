function [imin] = Find_Nearest_Point(x, y, path)
%Find_Nearest_Point - gets the point in the given path closest, using Euclidean
%Distance, to the given x and y coordinates


imin = 1;
min = (path(1,1)-x)^2 + (path(1,2)-y)^2;

for i = 2:length(path);
    if min > (path(i,1)-x)^2 + (path(i,2)-y)^2;
        min = (path(i,1)-x)^2 + (path(i,2)-y)^2;
        imin = i;
    end
end
end

