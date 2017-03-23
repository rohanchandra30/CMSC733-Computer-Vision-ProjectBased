function [C, R] = LinearPnP(X, x, K)
%% Inputs
% X and x: Nx3 and Nx2 matrices whose row represents correspondences between 
%          3D and 2D points, respectively.
% K: intrinsic parameter
%% Output
% C and R: camera pose (C;R)

%% Your Code goes here 
  N = size(x,1);
  
  if N >= 6
     X = [X ones(N,1)];
     x = [x ones(N,1)]';
     x_norm = K\x;
     P = mldivide(x_norm,X);
     R = P(:, 1:3);
     T = P(:, 4);
     [U,D,V] = svd(R);
      R = U*V';
     if det(R) == -1
      R = R.* -1;
     end
     C = (R\T).* -1;
  else
    fprintf('At least 6 correspondenses are required\n');
    C = zeros(3,1);
    R = zeros(3);
  end    
   
end


