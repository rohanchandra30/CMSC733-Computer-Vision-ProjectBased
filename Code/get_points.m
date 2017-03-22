function [a, b] = get_points(points, j ,k)

L = length(points);
for i = 1:L
    
    
    a(i,1) = points{i}(1);
    a(i,2) = points{i}(2);
    b(i,1) = points{i+L}(1);
    b(i,2) = points{i+L}(2);
end


end