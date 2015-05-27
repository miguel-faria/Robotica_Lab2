function [imin] = Find_Nearest_Point_Real(x, y, path, last_index)
%Find_Nearest_Point - gets the point in the given path closest, using Euclidean
%Distance, to the given x and y coordinates


imin = last_index;
min_val = (path(last_index,2)-x)^2 + (path(last_index,1)-y)^2;

max_index = min(last_index + 10,length(path));

for i = max(2,last_index+1):max_index;
    if min_val > (path(i,2)-x)^2 + (path(i,1)-y)^2;
        min_val = (path(i,2)-x)^2 + (path(i,1)-y)^2;
        imin = i;
    end
end
end

