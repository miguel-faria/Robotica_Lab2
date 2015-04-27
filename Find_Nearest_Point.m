function [imin] = Find_Nearest_Point(x, y, path)

imin=1;
min=(path(1,1)-x)^2+(path(1,2)-y)^2;

for i=2:length(path);
    if min>(path(i,1)-x)^2+(path(i,2)-y)^2;
        min=(path(i,1)-x)^2+(path(i,2)-y)^2;
        imin=i;
    end
end
end

