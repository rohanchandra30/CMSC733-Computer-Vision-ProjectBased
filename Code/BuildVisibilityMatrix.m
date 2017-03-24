function V = BuildVisibilityMatrix(Nimages, X)
% Input:
% traj: trajectory of camera poses
% Output:
% V: Visibility matrix

%% Your code goes here

V = zeros(Nimages, length(X));

for i = 1:Nimages
    for j = 1:length(X)
        
        if Rset{i}(3,:)*(Xset{i}(j,:)' - Cset{i})>0
            V(i,j) =  1;
        end
        
        
    end
end












end
