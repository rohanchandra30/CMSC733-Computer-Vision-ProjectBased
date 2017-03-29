function [Points,idx,F] = get_pointsandF_after_RANSAC_modified(points, Nimages)
%Nimages = 6;
% threshold = 4e-3;
threshold = 8e-3;

for i=1:Nimages-1
    idxi = find(cellfun(@(x) ~isempty(x), points(:,i)));
    for j = i+1:Nimages
        idxj = find(cellfun(@(x) ~isempty(x), points(:,j)));
        indx = intersect(idxi,idxj);
        if ~isempty(indx)
            [a1,a2,id,f] = GetInliersRANSAC_new(points(:,i),points(:,j), indx, threshold);
            Points{i,j} = [a1,a2];
            idx{i,j} = id;
            F{i,j} = f;
        end
    end
end



end