function [Points,F] = get_pointsandF_after_RANSAC(points)

Nimages = 6;
for i=1:Nimages-1
    for j = 1:Nimages
        if ~isempty(points{i,j})
            [a1,a2,id,f] = GetInliersRANSAC(points{i,j}(:,1:2),points{i,j}(:,3:4));
            Points{i,j} = [a1,a2];
            
            idx{i,j} = id;
            F{i,j} = f;
        end
    end
end



end