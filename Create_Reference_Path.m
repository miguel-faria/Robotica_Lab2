function [ path ] = Create_Reference_Path( speed, psi, x1, y1 )

    x(1) = x1;
    y(1) = y1;
    
    for i = 2:length(psi)
        x(i) = x(i-1) + speed*cos(psi(i));
        y(i) = y(i-1) - speed*sin(psi(i));
    end
    
    path = [x' y'];
end

