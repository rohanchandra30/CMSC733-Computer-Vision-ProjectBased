function [X] = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% Inputs:
% C1 and R1: the 1st camera pose
% C2 and R2: the 2nd camera pose
% x1 and x2: two Nx2 matrices whose row represents correspondence between the
%            1st and 2nd images where N is the number of correspondences.
%% Outputs:
% X: Nx3 matrix whose row represents 3D triangulated point.

%% Your Code goes here


P = K *[R1 (R1*C1).*-1 ];
P_dash = K *[R2 (R2*C2).*-1 ];

% P = [R1 C1];
% P_dash = [R2 C2];

% x1 goes with P. x2 goes with P_dash
for i = 1:size(x1,1)
    A = [x1(i,1)*P(3,:) - P(1,:);...
        x1(i,2)*P(3,:) - P(2,:);...
        x2(i,1)*P_dash(3,:) - P_dash(1,:);...
        x2(i,2)*P_dash(3,:) - P_dash(2,:)];
    
    
    [U,S,V] = svd(A);
    
    
    X{i} = V(:,end);
    
    X{i} = X{i}./X{i}(4);
    
    X{i} = X{i}(1:3);
    
end

X = cell2mat(X)';
end
