function F = EstimateFundamentalMatrix(X1,X2)
%% Input
% X1 and X2: Nx2 matrices whose row represents a correspondence
%% Output
% F: 3x3 Fundamental Matrix of Rank 2

%% Your Code goes here
 A = ones(size(X1,1),9);
 for i = 1: size(X1,1)
    x1 = X1(i,1);
    y1 = X1(i,2);
    xx = X2(i,1);
    yy = X2(i,2);
    A(i,:) = [x1*xx xx*y1 xx yy*x1 y1*yy yy x1 y1 1]; 
 end
 
 [U S V] = svd(A);
 F = V(:,9);
 F = reshape(F,3,3)';
 F = F./F(3,3);
 [A,B,C] = svd(F);
 B(3,3) = 0;
 B = B./B(1,1);
 F = A*B*C;
 
 
end
