function [Points,idx,F] = get_pointsandF_after_RANSAC_modified_EC(points)
Nimages = 6;

for i=1:2
    idxi = find(cellfun(@(x) ~isempty(x), points(:,i)));
    for j = i+1:i+1
        idxj = find(cellfun(@(x) ~isempty(x), points(:,j)));
        indx = intersect(idxi,idxj);
        if ~isempty(indx)
            [a1,a2,id,f] = GetInliersRANSAC_new(points(:,i),points(:,j), indx);
            Points{i,j} = [a1,a2];
            idx{i,j} = id;
            F{i,j} = f;
        end
    end
end



end