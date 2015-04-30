function [ type ] = Type_Map_Matrix( map_matrix)
%Type_Map_Matrix - Gives the type for the map matrix (section of the map):
%   - type 0: the matrix only has navigatable points
%   - type 1: the matrix only has non-navigatable points
%   - type 2: the matrix has a mix of navigatable and non-navigatable
%   points

if all(map_matrix == 0)
    type = 1;
elseif (isempty(map_matrix) == 1)
    type = 1;
elseif all(map_matrix == 255)
    type = 0;
else
    type = 2;
end

end

