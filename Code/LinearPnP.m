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
     A = [];
     for i = 1:6
%         x_p = [x(i,:)'; 1];
%         X_p = [X(i,:)'; 1];
%         Temp = [ 0 -x_p(3) x_p(2);...
%             x_p(3) 0 -x_p(1);...
%             -x_p(2) x_p(1) 0]*[X_p' zeros(1,8);...
%                             zeros(1,4) X_p' zeros(1,4);...
%                                        zeros(1,8) X_p'];
%          A = [A; Temp];                         
         A = [ A;
              -X(i,1), -X(i,2), -X(i,3), -1, 0, 0, 0, 0, x_norm(1,i).*X(i,1), x_norm(1,i).*X(i,2), x_norm(1,i).*X(i,3), x_norm(1,i);
              0, 0, 0, 0, -X(i,1), -X(i,2), -X(i,3), -1, x_norm(2,i).*X(i,1) x_norm(2,i).*X(i,2) x_norm(2,i).*X(i,3) x_norm(2,i)];
     end    
     [U, S, M] = svd(A);
     P = reshape(M(:,12), 4,3)';
% %      P = K\P;
     R = P(:, 1:3);
     T = P(:, 4);
     [U,S,V] = svd(R);
     R = U*V';
     if det(R) == -1
      R = R.* -1;
     end
     C = (R\T).* -1;
  else
    fprintf('At least 6 correspondenses are required\n');
    C = zeros(3,1);
    R = eye(3);
  end    
   
end


