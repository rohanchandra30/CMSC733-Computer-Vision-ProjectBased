function V = BuildVisibilityMatrix(Xset, Rset, Cset, Nimages_so_far)
% Input:
% traj: trajectory of camera poses
% Output:
% V: Visibility matrix

%% Your code goes here
X = [];
for k = 1:numel(Xset)
   if numel(Xset{k})~=0 
   X = [X; Xset{k}]; 
   end
    
end
V = zeros(Nimages_so_far, length(X));

for i = 1:Nimages_so_far
    for j = 1:length(X)
        
        if Rset{i}(3,:)*(X(j,:)' - Cset{i})>0
            V(i,j) =  1;
        end
        
        
    end
end












end
