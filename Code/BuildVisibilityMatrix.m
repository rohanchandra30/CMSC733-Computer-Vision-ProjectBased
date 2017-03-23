function V = BundleAdjustment(Nimages, X)
% Input:
% traj: trajectory of camera poses
% Output:
% V: Visibility matrix

%% Your code goes here


for i = 1:Nimages
    for j = 1:length(X)
        
        if Rset{i}(3,:)*(Xset{i}(j,:)' - Cset{i})>0
            V(i,j) =  
        end
        
        
    end
end












end
